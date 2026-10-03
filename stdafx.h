// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <windows.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <tchar.h>
#include <vector>
#include <omp.h>
#include <queue>
#include <algorithm>
#include <numeric>
#include "conio.h"
#include <mmsystem.h>
#include <concurrent_queue.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "FlyCapture2.h"
#include <NIDAQmx.h>

#include "pgrcam.h"
#include "fvfmfwriter.h"
#include "avfmfwriter.h"
#include "daq.h"
#include "utility.h"
//#include "arduino.h"

#include "sapclassbasic.h"
#include "ExampleUtils.h"

#define GALVO_Y_HEIGHT 68.167			//in mm
#define GALVO_XY_DIST 15.174			//in mm
#define GALVO_X_MIRROR_ANGLE 15			//in degrees
//#define GALVO_STEP_SIZE 0.0000075		// doubled the step size to maintain similar manual speed control (due to spectre/meltdown processor slowdown)
#define GALVO_STEP_SIZE 0.000015		

#define XVOLTPERDEGREE 0.51
#define YVOLTPERDEGREE 0.514

//#define XOFFSET -0.25		// x-offset for centering galvo to target (in volts)
//#define YOFFSET -0.315		// y-offset for centering galvo to target (in volts)

#define XOFFSET -0.00		// x-offset for centering galvo to target (in volts)
#define YOFFSET -0.00		// y-offset for centering galvo to target (in volts)

#define SCALEX 0.0008
#define SCALEY 0.0008

#define NFLIES 1
#define NLOSTFRAMES 5

#define RECTIMEINSEC	60*1
#define MAXFVRECFRAMES 1000*RECTIMEINSEC
#define MAXAVRECFRAMES 50*RECTIMEINSEC

#define FLASHFRAMES 5

#define MIN_MARKER_SIZE 10
#define MAX_MARKER_SIZE 500

#define NBEADS 3	

//#define Z_PERIOD 50
//#define Z_EPSILON 5.0
//#define Z_STEP_COARSE 2.0
//#define Z_STEP_FINE 1.0
//#define Z_STEP_HOME 100.0

// =========================================================================
// Default Focus Startup Voltages (Liquid Lenses AO2 & AO3)
// =========================================================================
#define DEFAULT_LENS1_VOLTAGE  5.20f   // Default AO2 voltage (V) - update after F6 test
#define DEFAULT_LENS2_VOLTAGE  5.20f   // Default AO3 voltage (V) - update after F6 test
#define DEFAULT_BASE_FM        28.60   // Default baseline image focus score

// =========================================================================
// Arena head tracking parameters
// =========================================================================

// Total centroid displacement required before initial head/tail polarity
// is determined from movement direction.
#define HEAD_INIT_DISPLACEMENT 3.0f

// Weight of the newest body-axis direction.
// 1.0 = fastest response, no angular smoothing.
// Lower values = smoother but slower response.
#define HEAD_DIR_ALPHA 0.75f

// Fraction of the centroid-to-head-contour distance used as the target.
// 1.0 = exactly on the contour boundary.
// 0.85 = slightly inside the head side of the body.
#define HEAD_TARGET_RATIO 0.85f

// TODO: reference additional headers your program requires here
