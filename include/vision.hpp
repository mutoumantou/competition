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
    void on_spin_binaryThreshold_changed (GtkEditable *editable, gpointer user_data);

}

/* Functions */
void camera_activate (void);
void camera_deactivate (void);
void get_present_image ( Mat * container );

void return_center_pt_info ( Point *robot, Point *cargo );  // return current position information of robot and cargo to controller

#endif
