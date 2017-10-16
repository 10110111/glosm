/*
 * Copyright (C) 2010-2012 Dmitry Marakasov
 *
 * This file is part of glosm.
 *
 * glosm is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * glosm is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with glosm.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "GlosmViewer.hh"

#include <glosm/Math.hh>
#include <glosm/MercatorProjection.hh>
#include <glosm/SphericalProjection.hh>
#include <glosm/Timer.hh>
#include <glosm/CheckGL.hh>
#include <glosm/geomath.h>

#include <glosm/util/gl.h>

#include <getopt.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

#include <GL/glu.h>

GlosmViewer::GlosmViewer() : projection_(MercatorProjection()), viewer_(new FirstPersonViewer) {
	screenw_ = screenh_ = 1;
	nframes_ = 0;

	movementflags_ = 0;
	speed_ = 200.0f;
	lockheight_ = 0;

	drag_ = slow_ = fast_ = false;

#if defined(WITH_TOUCHPAD)
	mouse_capture_ = false;
#else
	mouse_capture_ = true;
#endif

	ground_shown_ = true;
	detail_shown_ = true;
	gpx_shown_ = true;
	terrain_shown_ = true;

	no_glew_check_ = false;

	start_lon_ = start_lat_ = start_ele_ = start_yaw_ = start_pitch_ = nan("");
}

void GlosmViewer::Usage(int status, bool detailed, const char* progname) {
	fprintf(stderr, "Usage: %s [-sfh] [-t <path>] [-l lon,lat,ele,yaw,pitch] <file.osm|-> [file.gpx ...]\n", progname);
	if (detailed) {
		fprintf(stderr, "Options:\n");
		//               [==================================72==================================]
		fprintf(stderr, "  -h       - show this help\n");
		fprintf(stderr, "  -s       - use spherical projection instead of mercator\n");
		fprintf(stderr, "  -t path  - add terrain layer, argument specifies path to directory\n");
		fprintf(stderr, "             with SRTM data (*.hgt files)\n");
		fprintf(stderr, "  -l ...   - set initial viewer's location and direction\n");
		fprintf(stderr, "             argument is comma-separated list of longitude, latitude,\n");
		fprintf(stderr, "             elevation, pitch and yaw, each of those may be empty for\n");
		fprintf(stderr, "             program default (e.g. -l ,,100.0,, or -l ,,100.0). Units\n");
		fprintf(stderr, "             are degrees and meters\n");
#if defined(WITH_GLEW)
		fprintf(stderr, "  -f       - ignore glew errors\n");
#endif
	}
	exit(status);
}

void GlosmViewer::Init(int argc, char** argv) {
	/* argument parsing */
	int c;
	const char* progname = argv[0];
	const char* srtmpath = NULL;
	while ((c = getopt(argc, argv, "sfht:l:")) != -1) {
		switch (c) {
		case 's': projection_ = SphericalProjection(); break;
		case 't': srtmpath = optarg; break;
		case 'l': {
					  int n = 0;
					  char* start = optarg;
					  char* end;
					  char* endptr;
					  do {
						  if ((end = strchr(start, ',')) != NULL)
							  *end = '\0';

						  double val = strtod(start, &endptr);

						  if (endptr != start) {
							  switch (n) {
							  case 0: start_lon_ = val; break;
							  case 1: start_lat_ = val; break;
							  case 2: start_ele_ = val; break;
							  case 3: start_yaw_ = val; break;
							  case 4: start_pitch_ = val; break;
							  }
						  }

						  n++;
						  start = end + 1;
					  } while(end != NULL);
				  } break;
#if defined(WITH_GLEW)
		case 'f': no_glew_check_ = true; break;
#endif
		case 'h': Usage(0, true, progname); break;
		default:
			Usage(1, false, progname);
		}
	}

	argc -= optind;
	argv += optind;

	/* load data */
	for (int narg = 0; narg < argc; ++narg) {
		std::string file = argv[narg];

		if (file == "-" || file.rfind(".osm") == file.length() - 4) {
			fprintf(stderr, "Loading %s as OSM...\n", file == "-" ? "stdin" : argv[narg]);
			if (osm_datasource_.get() == NULL) {
				Timer t;
				osm_datasource_.reset(new PreloadedXmlDatasource);
				osm_datasource_->Load(argv[narg]);
				fprintf(stderr, "Loaded in %.3f seconds\n", t.Count());
			} else {
				fprintf(stderr, "Only single OSM file may be loaded at once, skipped\n");
			}
		} else if (file.rfind(".gpx") == file.length() - 4) {
			fprintf(stderr, "Loading %s as GPX...\n", argv[narg]);
			if (gpx_datasource_.get() == NULL)
				gpx_datasource_.reset(new PreloadedGPXDatasource);

			Timer t;
			gpx_datasource_->Load(argv[narg]);
			fprintf(stderr, "Loaded in %.3f seconds\n", t.Count());
		} else {
			fprintf(stderr, "Not loading %s - unknown file type\n", argv[narg]);
		}
	}

	if (srtmpath) {
		heightmap_datasource_.reset(new SRTMDatasource(srtmpath));
		viewer_->SetHeightmapDatasource(heightmap_datasource_.get());
	} else {
		heightmap_datasource_.reset(new DummyHeightmap());
	}

	if (osm_datasource_.get() == NULL)
		throw Exception() << "no osm dump specified";

	gettimeofday(&curtime_, NULL);
	prevtime_ = curtime_;
	fpstime_ = curtime_;
}

void GlosmViewer::InitGL() {
	ShowCursor(!mouse_capture_);

#if defined(WITH_GLEW)
	GLenum err = glewInit();
	if (err != GLEW_OK)
		throw Exception() << "Cannot init glew: " << glewGetErrorString(err);
	const char *gl_requirements = "GL_VERSION_1_5";
	if (!glewIsSupported(gl_requirements)) {
		Exception e;
		e << "Minimal OpenGL requirements (" << gl_requirements << ") not met, unable to continue";
		if (no_glew_check_)
			fprintf(stderr, "(ignored) %s\n", e.what());
		else
			throw e;
	}
#endif
	CheckGL();

	geometry_generator_.reset(new GeometryGenerator(*osm_datasource_, *heightmap_datasource_));
	ground_layer_.reset(new GeometryLayer(projection_, *geometry_generator_));
	detail_layer_.reset(new GeometryLayer(projection_, *geometry_generator_));

	ground_layer_->SetLevel(9);
	ground_layer_->SetRange(1000000.0);
	ground_layer_->SetFlags(GeometryDatasource::GROUND);
	ground_layer_->SetHeightEffect(false);
	ground_layer_->SetSizeLimit(32*1024*1024);

	detail_layer_->SetLevel(12);
	detail_layer_->SetRange(10000.0);
	detail_layer_->SetFlags(GeometryDatasource::DETAIL);
	detail_layer_->SetHeightEffect(true);
	detail_layer_->SetSizeLimit(96*1024*1024);

	if (gpx_datasource_.get()) {
		gpx_layer_.reset(new GPXLayer(projection_, *gpx_datasource_, *heightmap_datasource_));
		gpx_layer_->SetLevel(10);
		gpx_layer_->SetRange(10000.0);
		gpx_layer_->SetHeightEffect(true);
		gpx_layer_->SetSizeLimit(32*1024*1024);
	}

	if (heightmap_datasource_.get()) {
		terrain_layer_.reset(new TerrainLayer(projection_, *heightmap_datasource_));
		terrain_layer_->SetLevel(12);
		terrain_layer_->SetRange(20000.0);
		terrain_layer_->SetHeightEffect(false);
		terrain_layer_->SetSizeLimit(32*1024*1024);
	}

	Vector3i startpos = geometry_generator_->GetCenter();
	osmint_t startheight = fabs((float)geometry_generator_->GetBBox().top - (float)geometry_generator_->GetBBox().bottom) / GEOM_LONSPAN * WGS84_EARTH_EQ_LENGTH * GEOM_UNITSINMETER / 10.0;
	float startyaw = 0;
	float startpitch = -M_PI_4;

	if (!std::isnan(start_lon_))
		startpos.x = start_lon_ * GEOM_UNITSINDEGREE;
	if (!std::isnan(start_lat_))
		startpos.y = start_lat_ * GEOM_UNITSINDEGREE;
	if (!std::isnan(start_ele_))
		startheight = start_ele_ * GEOM_UNITSINMETER;
	if (!std::isnan(start_yaw_))
		startyaw = start_yaw_;
	if (!std::isnan(start_pitch_))
		startpitch = start_pitch_;

	viewer_->SetPos(Vector3i(startpos, startheight));
	viewer_->SetRotation(startyaw, startpitch);
#if defined(WITH_TOUCHPAD)
	lockheight_ = startheight;
#endif
}

namespace Sun
{
struct SunPos
{
    double azimuth;
    double elevation;
};

static const double PI=M_PI;
typedef double my;
double to360range(double num)
{
    my $num=num;

    if($num > 360) return $num - floor($num/360) * 360;
    if($num < 0) return $num + (floor(-$num/360) + 1)*360;
    return $num;
}

SunPos calcAzimuthAndElevation(int Y, int M, int D, double localTZOffset, double latitude, double longitude, int hours, int minutes, int seconds)
{
    using namespace std;
    my $Y=Y, $M=M, $D=D, $localTZOffset=localTZOffset, $latitude=latitude, $longitude=longitude, $hours=hours, $minutes=minutes, $seconds=seconds;

    // 1-based day number since Jan 1 2000
    my $d = 367*$Y - floor(7*($Y + floor(($M+9)/12))/4) + floor(275*$M/9) + $D - 730530;

    // longitude of perihelion
    my $w = 282.9404 + 4.70935*pow(10,-5)*$d;
    // eccentricity
    my $e = 0.016709 - 1.151*pow(10,-9)*$d;
    // mean anomaly
    $M = to360range(356.0470 + 0.9856002585*$d);
    // obliquity of the ecliptic
    my $oblecl = 23.4393 - 3.563*pow(10,-7)*$d;
    // mean longitude
    my $L = to360range($w + $M);

    // eccentric anomaly
    my $E = $M + (180/PI)*$e*sin($M*PI/180)*(1+$e*cos($M*PI/180));

    // rectangular coordinates in the plane of the ecliptic, where the X axis points towards the perihelion
    my $x = cos($E*PI/180)-$e;
    my $y = sin($E*PI/180)*sqrt(1-$e*$e);

    my $r = sqrt($x*$x+$y*$y);
    my $v = (180/PI)*atan2($y, $x);

    my $lon = to360range($v + $w);

    // ecliptic rectangular coordinates
    $x = $r * cos($lon*PI/180);
    $y = $r * sin($lon*PI/180);
    my $z = 0.0;

    // rotate to equatorial coordinates
    my $xequat = $x;
    my $yequat = $y * cos($oblecl*PI/180) + $z * sin($oblecl*PI/180);
    my $zequat = $y * sin($oblecl*PI/180) + $z * cos($oblecl*PI/180);

    // convert to RA and Declination
    my $RA = (180/PI) * atan2($yequat, $xequat);
    my $Decl = (180/PI) * asin( $zequat/$r );

    // point of observation longitude
    my $plon = $longitude;

    // Sidereal Time at the Greenwich meridian at 00:00 right now
    my $GMST0 = $L/15 + 12;
    my $UT = $hours+$minutes/60+$seconds/3600-$localTZOffset;
    my $SIDTIME = $GMST0 + $UT + $plon/15;
    $SIDTIME = $SIDTIME - 24 * floor($SIDTIME/24);

    // hour angle
    my $HA = to360range(15*($SIDTIME - $RA/15));

    $x = cos($HA*PI/180)*cos($Decl*PI/180);
    $y = sin($HA*PI/180)*cos($Decl*PI/180);
    $z = sin($Decl*PI/180);

    // point of observation latitude
    my $plat = $latitude;

    my $xhor = $x * sin($plat*PI/180) - $z * cos($plat*PI/180);
    my $yhor = $y;
    my $zhor = $x * cos($plat*PI/180) + $z * sin($plat*PI/180);

    my $azimuth = to360range(atan2($yhor, $xhor)*(180/PI) + 180);
    my $elevation = asin($zhor)*(180/PI);

    SunPos pos={$azimuth,$elevation};
    return pos;
}

void drawSun(GLUquadric* quadric, double azimuth, double elevation, double scale=1, int slices=30)
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glColor3f(1,0.5,0);
    glRotated(azimuth, 0,0,1);
    glRotated(elevation, 1,0,0);
    glTranslated(0,10,0);
    gluSphere(quadric,0.1*scale, slices, slices);
    glPopMatrix();
}

void draw(FirstPersonViewer const& viewer, Projection const& proj)
{
    viewer.SetupViewerMatrix(proj);
    const Vector3i pos=viewer.GetPos(proj);
    const double pitch=viewer.GetPitch();
    const double yaw=viewer.GetYaw();

    timeval tv;
    if(gettimeofday(&tv,NULL)==-1)
    {
        perror("gettimeofday");
        return;
    }
    struct tm* tm;
    if(!(tm=localtime(&tv.tv_sec)))
    {
        perror("localtime");
        return;
    }
    const double longitude=double(pos.x)/GEOM_UNITSINDEGREE;
    const double latitude=double(pos.y)/GEOM_UNITSINDEGREE;
    const double localTZOffset=-timezone/3600.;
    const SunPos sunPos=calcAzimuthAndElevation(tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, localTZOffset, latitude, longitude, tm->tm_hour, tm->tm_min, tm->tm_sec);
    std::cerr << "sun azimuth: " << sunPos.azimuth << "°, elevation: " << sunPos.elevation << "\n";

    GLUquadric* quadric=gluNewQuadric();
    drawSun(quadric, -sunPos.azimuth,sunPos.elevation);

    for(int h=0;h<24;++h)
    {
        for(int m=0;m<60;m+=1)
        {
            const SunPos sunPos=calcAzimuthAndElevation(tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, localTZOffset, latitude, longitude, h, m, 0);
            drawSun(quadric,-sunPos.azimuth,sunPos.elevation, 0.1, 4);
        }
    }
    gluDeleteQuadric(quadric);
}

}

void GlosmViewer::Render() {
	/* update scene */
	gettimeofday(&curtime_, NULL);
	float dt = (float)(curtime_.tv_sec - prevtime_.tv_sec) + (float)(curtime_.tv_usec - prevtime_.tv_usec)/1000000.0f;

	/* render frame */
	glClearColor(0.0, 0.5, 1.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Sun::draw(*viewer_,projection_);

	if (ground_shown_) {
		ground_layer_->GarbageCollect();
		ground_layer_->LoadLocality(*viewer_);
		ground_layer_->Render(*viewer_);
	}

	if (detail_shown_) {
		detail_layer_->GarbageCollect();
		detail_layer_->LoadLocality(*viewer_);
		detail_layer_->Render(*viewer_);
	}

	if (gpx_shown_ && gpx_layer_.get()) {
		gpx_layer_->GarbageCollect();
		gpx_layer_->LoadLocality(*viewer_);
		gpx_layer_->Render(*viewer_);
	}

	if (terrain_shown_ && terrain_layer_.get()) {
		terrain_layer_->GarbageCollect();
		terrain_layer_->LoadLocality(*viewer_);
		terrain_layer_->Render(*viewer_);
	}

	glFlush();
	Flip();

	/* movement */
	if (movementflags_) {
		float myspeed = speed_;
		float height = viewer_->MutablePos().z / GEOM_UNITSINMETER;

		/* don't scale down under 100 meters */
		if (height > 100.0)
			myspeed *= height / 100.0;

		if (fast_)
			myspeed *= 5.0;
		if (slow_)
			myspeed /= 5.0;

		viewer_->Move(movementflags_, myspeed, dt);
	}
	if (lockheight_ != 0)
		viewer_->MutablePos().z = lockheight_;

	/* update FPS */
	float fpst = (float)(curtime_.tv_sec - fpstime_.tv_sec) + (float)(curtime_.tv_usec - fpstime_.tv_usec)/1000000.0f;

	if (fpst > 10.0) {
		fprintf(stderr, "FPS: %.3f\n", (float)nframes_/fpst);
		fpstime_ = curtime_;
		nframes_ = 0;
	}

	prevtime_ = curtime_;
	nframes_++;

#if !defined(DEBUG_FPS)
	/* frame limiter */
	usleep(10000);
#endif
}

void GlosmViewer::Resize(int w, int h) {
	if (w <= 0)
		w = 1;
	if (h <= 0)
		h = 1;

	screenw_ = w;
	screenh_ = h;

	float wanted_fov = 70.0f/180.0f*M_PI;
	float wanted_aspect = 4.0f/3.0f;

	float fov, aspect;

	if ((float)w/(float)h > wanted_aspect) { // wider than wanted
		fov = wanted_fov;
	} else { // narrower than wanted
		float wanted_h = (float)w / wanted_aspect;
		fov = 2.0f * atanf((float)h / wanted_h * tanf(wanted_fov/2.0f));
	}
	fov = wanted_fov;
	aspect = (float)w/(float)h;

	glViewport(0, 0, w, h);

	viewer_->SetFov(fov);
	viewer_->SetAspect(aspect);

	WarpCursor(w/2, h/2);
}

void GlosmViewer::KeyDown(int key) {
	switch (key) {
	case 27: case 'q':
		exit(0);
		break;
	case 'w': case KEY_UP:
		movementflags_ |= FirstPersonViewer::FORWARD;
		break;
	case 's': case KEY_DOWN:
		movementflags_ |= FirstPersonViewer::BACKWARD;
		break;
	case 'a': case KEY_LEFT:
		movementflags_ |= FirstPersonViewer::LEFT;
		break;
	case 'd': case KEY_RIGHT:
		movementflags_ |= FirstPersonViewer::RIGHT;
		break;
	case 'c':
		movementflags_ |= FirstPersonViewer::LOWER;
		break;
	case ' ':
		movementflags_ |= FirstPersonViewer::HIGHER;
		break;
	case 'l':
		lockheight_ = (lockheight_ == 0 ? viewer_->MutablePos().z : 0);
		break;
	case 'h':
		lockheight_ = (lockheight_ == 0 ? 1.75 * GEOM_UNITSINMETER : 0);
		break;
	case '+':
		speed_ *= 5.0f;
		break;
	case '-':
		speed_ /= 5.0f;
		break;
	case '1':
		ground_shown_ = !ground_shown_;
		break;
	case '2':
		detail_shown_ = !detail_shown_;
		break;
	case '3':
		gpx_shown_ = !gpx_shown_;
		break;
	case '4':
		terrain_shown_ = !terrain_shown_;
		break;
	case KEY_SHIFT:
		fast_ = true;
		break;
	case KEY_CTRL:
		slow_ = true;
		break;
	default:
		break;
	}
}

void GlosmViewer::KeyUp(int key) {
	switch (key) {
	case 'w': case KEY_UP:
		movementflags_ &= ~FirstPersonViewer::FORWARD;
		break;
	case 's': case KEY_DOWN:
		movementflags_ &= ~FirstPersonViewer::BACKWARD;
		break;
	case 'a': case KEY_LEFT:
		movementflags_ &= ~FirstPersonViewer::LEFT;
		break;
	case 'd': case KEY_RIGHT:
		movementflags_ &= ~FirstPersonViewer::RIGHT;
		break;
	case 'c':
		movementflags_ &= ~FirstPersonViewer::LOWER;
		break;
	case ' ':
		movementflags_ &= ~FirstPersonViewer::HIGHER;
		break;
	case KEY_SHIFT:
		fast_ = false;
		break;
	case KEY_CTRL:
		slow_ = false;
		break;
	default:
		break;
	}
}

void GlosmViewer::MouseMove(int x, int y) {
	if (drag_) {
		int dx = x - drag_start_pos_.x;
		int dy = y - drag_start_pos_.y;

		float yawdelta = -(float)dx / (float)screenw_ * viewer_->GetFov() * viewer_->GetAspect();
		float pitchdelta = (float)dy / (float)screenh_ * viewer_->GetFov();

		viewer_->SetRotation(drag_start_yaw_ + yawdelta, drag_start_pitch_ + pitchdelta);
	}

	if (mouse_capture_) {
		int dx = x - screenw_/2;
		int dy = y - screenh_/2;

		float yawdelta = (float)dx / 500.0;
		float pitchdelta = -(float)dy / 500.0;

		viewer_->Rotate(yawdelta, pitchdelta, 1.0);

		if (dx != 0 || dy != 0)
			WarpCursor(screenw_/2, screenh_/2);
	}
}

void GlosmViewer::MouseButton(int button, bool pressed, int x, int y) {
	if (button == BUTTON_RIGHT && pressed) {
		mouse_capture_ = !mouse_capture_;
		ShowCursor(!mouse_capture_);
	}

	if (!mouse_capture_ && button == BUTTON_LEFT) {
		drag_ = pressed;
		if (pressed) {
			drag_start_pos_.x = x;
			drag_start_pos_.y = y;
			drag_start_yaw_ = viewer_->GetYaw();
			drag_start_pitch_ = viewer_->GetPitch();
		}
	}
}
