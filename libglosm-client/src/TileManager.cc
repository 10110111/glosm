/*
 * Copyright (C) 2010-2011 Dmitry Marakasov
 *
 * This file is part of glosm.
 *
 * glosm is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * glosm is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with glosm.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <glosm/TileManager.hh>

#if defined(__APPLE__)
#	include <OpenGL/gl.h>
#else
#	include <GL/gl.h>
#endif

#include <glosm/Viewer.hh>
#include <glosm/Geometry.hh>
#include <glosm/GeometryDatasource.hh>
#include <glosm/Tile.hh>

#include <stdexcept>
#include <cassert>

TileManager::TileId::TileId(int lev, int xx, int yy) : level(lev), x(xx), y(yy) {
}

bool TileManager::TileId::operator<(const TileId& other) const {
	if (level < other.level) return true;
	if (level > other.level) return false;
	if (x < other.x) return true;
	if (x > other.x) return false;
	return y < other.y;
}

TileManager::TileData::TileData(): tile(NULL), geometry(NULL), generation(0), loading(false) {
}

TileManager::TileData::~TileData() {
	delete tile;
	delete geometry;
}

TileManager::TileManager(const Projection projection, const GeometryDatasource& ds): projection_(projection), datasource_(ds) {
	if (pthread_mutex_init(&tiles_mutex_, 0) != 0)
		throw std::runtime_error("pthread_mutex_init failed");

	if (pthread_cond_init(&tiles_cond_, 0) != 0) {
		pthread_mutex_destroy(&tiles_mutex_);
		throw std::runtime_error("pthread_cond_init failed");
	}

	if (pthread_create(&loading_thread_, NULL, LoadingThreadFuncWrapper, (void*)this) != 0) {
		pthread_cond_destroy(&tiles_cond_);
		pthread_mutex_destroy(&tiles_mutex_);
		throw std::runtime_error("pthread_create failed");
	}

	target_level_ = 0;
	generation_ = 0;
	thread_die_flag_ = false;
}

TileManager::~TileManager() {
	thread_die_flag_ = true;
	pthread_cond_signal(&tiles_cond_);

	pthread_join(loading_thread_, NULL);
	pthread_cond_destroy(&tiles_cond_);
	pthread_mutex_destroy(&tiles_mutex_);
}

/*
 * recursive quadtree processing
 */

void TileManager::LoadTiles(const BBoxi& bbox, int flags, int level, int x, int y) {
	if (level == target_level_) {
		TilesMap::iterator thistile = tiles_.find(TileId(level, x, y));
		if (thistile == tiles_.end())
			thistile = tiles_.insert(std::make_pair(TileId(level, x, y), TileData()));

		/* update generation */
		thistile->second.generation = generation_;

		/* already loaded or loading, skip */
		if (thistile->second.tile || thistile->second.loading)
			return;

		/* geometry loaded by thread, turn into tile */
		if (thistile->second.geometry) {
			/* XXX: projection and VBO loading still
			 * takes some time, maybe we need to limit
			 * this per-frame */
			thistile->second.tile = SpawnTile(*thistile->second.geometry);
			delete thistile->second.geometry;
			thistile->second.geometry = NULL;
			return;
		}

		/* load immediately for sync case */
		if (flags && SYNC) {
			BBoxi bbox = BBoxi::ForGeoTile(level, x, y);
			Geometry geom(bbox);
			datasource_.GetGeometry(geom, bbox);
			thistile->second.tile = SpawnTile(geom);
			return;
		}

		/* in other case, it will be loaded by thread */
		pthread_cond_signal(&tiles_cond_);
		return;
	}

	/* children */
	for (int d = 0; d < 4; ++d) {
		int xx = x * 2 + d % 2;
		int yy = y * 2 + d / 2;
		if (BBoxi::ForGeoTile(level + 1, xx, yy).Intersects(bbox)) {
			LoadTiles(bbox, flags, level + 1, xx, yy);
		}
	}
}

/*
 * loading queue - related
 */

void TileManager::LoadingThreadFunc() {
	while (!thread_die_flag_) {
		pthread_mutex_lock(&tiles_mutex_);

		TilesMap::iterator to_load = tiles_.end();
		for (TilesMap::iterator i = tiles_.begin(); i != tiles_.end(); ++i) {
			if (!i->second.loading && !i->second.tile && !i->second.geometry) {
				to_load = i;
				i->second.loading = true;
				break;
			}
		}

		/* found nothing, sleep */
		if (to_load == tiles_.end()) {
			pthread_cond_wait(&tiles_cond_, &tiles_mutex_);
			pthread_mutex_unlock(&tiles_mutex_);
			continue;
		}

		pthread_mutex_unlock(&tiles_mutex_);

		/* load tile */
		BBoxi bbox = BBoxi::ForGeoTile(to_load->first.level, to_load->first.x, to_load->first.y);
		to_load->second.geometry = new Geometry(bbox);
		datasource_.GetGeometry(*(to_load->second.geometry), bbox);

		to_load->second.loading = false;
	}
}

void* TileManager::LoadingThreadFuncWrapper(void* arg) {
	static_cast<TileManager*>(arg)->LoadingThreadFunc();
	return NULL;
}

/*
 * protected interface
 */

void TileManager::Render(const Viewer& viewer) const {
	std::vector<Tile*> tiles;

	/* create array of renderable tiles */
	pthread_mutex_lock(&tiles_mutex_);
	tiles.reserve(tiles_.size());
	for (TilesMap::const_iterator i = tiles_.begin(); i != tiles_.end(); ++i)
		if (i->second.tile && i->second.generation == generation_)
			tiles.push_back(i->second.tile);
	pthread_mutex_unlock(&tiles_mutex_);

	/* and render them */
	for (std::vector<Tile*>::iterator i = tiles.begin(); i != tiles.end(); ++i) {
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();

		/* prepare modelview matrix for the tile: position
		 * it in the right place given that viewer is always
		 * at (0, 0, 0) */
		Vector3f offset = projection_.Project((*i)->GetReference(), Vector2i(viewer.GetPos(projection_))) +
				projection_.Project(Vector2i(viewer.GetPos(projection_)), viewer.GetPos(projection_));

		glTranslatef(offset.x, offset.y, offset.z);

		/* same for rotation */
		Vector3i ref = (*i)->GetReference();
		Vector3i pos = viewer.GetPos(projection_);

		/* normal at tile's reference point */
		Vector3d refnormal = (
				(Vector3d)projection_.Project(Vector3i(ref.x, ref.y, std::numeric_limits<osmint_t>::max()), pos) -
				(Vector3d)projection_.Project(Vector3i(ref.x, ref.y, 0), pos)
			).Normalized();

		/* normal at reference point projected to equator */
		Vector3d refeqnormal = (
				(Vector3d)projection_.Project(Vector3i(ref.x, 0, std::numeric_limits<osmint_t>::max()), pos) -
				(Vector3d)projection_.Project(Vector3i(ref.x, 0, 0), pos)
			).Normalized();

		/* normal at north pole */
		Vector3d polenormal = (
				(Vector3d)projection_.Project(Vector3i(ref.x, 900000000, std::numeric_limits<osmint_t>::max()), pos) -
				(Vector3d)projection_.Project(Vector3i(ref.x, 900000000, 0), pos)
			).Normalized();

		/* XXX: IsValid() check basically detects
		 * MercatorProjection and does no rotation for it.
		 * While is's ok for now, this may need more generic
		 * approach in future */
		if (polenormal.IsValid()) {
			Vector3d side = refnormal.CrossProduct(polenormal).Normalized();

			glRotatef((double)((osmlong_t)ref.y - (osmlong_t)pos.y) / 10000000.0, side.x, side.y, side.z);
			glRotatef((double)((osmlong_t)ref.x - (osmlong_t)pos.x) / 10000000.0, polenormal.x, polenormal.y, polenormal.z);
		}

		(*i)->Render();

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}
}

/*
 * public interface
 */

void TileManager::SetTargetLevel(int level) {
	target_level_ = level;
}

void TileManager::RequestVisible(const BBoxi& bbox, int flags) {
	if (!(flags & NOGENBUMP))
		++generation_;

	if (flags & EXPLICIT) {
		Geometry geom(bbox);
		datasource_.GetGeometry(geom, bbox);

		TilesMap::iterator thistile = tiles_.insert(std::make_pair(TileId(0, 0, 0), TileData()));
		thistile->second.tile = SpawnTile(geom);
		thistile->second.generation = generation_;
	} else {
		pthread_mutex_lock(&tiles_mutex_);
		LoadTiles(bbox, flags);
		pthread_mutex_unlock(&tiles_mutex_);
	}
}

void TileManager::GarbageCollect() {
	pthread_mutex_lock(&tiles_mutex_);
	for (TilesMap::iterator i = tiles_.begin(); i != tiles_.end(); ) {
		if (!i->second.loading && i->second.generation != generation_) {
			TilesMap::iterator tmp = i++;
			tiles_.erase(tmp);
		} else {
			i++;
		}
	}
	pthread_mutex_unlock(&tiles_mutex_);
}