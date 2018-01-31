#include "callbacks.hpp"

void on_toggle_camera_stream_toggled (GtkToggleButton *togglebutton, gpointer data) {
    printf("this is a test.\n");
/*
    int d = gtk_toggle_button_get_active (togglebutton);
    if (d) {
        camera_activate ();
    } else {
        camera_deactivate ();
    }
    my_sleep(100);
    //fCam = d;
    */
}
