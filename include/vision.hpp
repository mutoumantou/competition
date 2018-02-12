#ifndef VISION
#define VISION

#include "general_header.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "FWcamera.hpp"

using namespace cv;

/* GUI callback functions */
extern "C" {
    void on_toggle_arena_toggled (GtkToggleButton *togglebutton, gpointer data);    // show/hide digital arena
}

/* Functions */
void camera_activate (void);
void camera_deactivate (void);
void get_present_image ( Mat * container );

#endif
