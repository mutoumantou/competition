#ifndef VISION
#define VISION

#include "general_header.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "FWcamera.hpp"

using namespace cv;

/* Functions */
void camera_activate (void);
void camera_deactivate (void);

#endif
