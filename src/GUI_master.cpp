#include "GUI_master.hpp"

static int fThread = 0;             // flag for GUI update running
static int fCam    = 0;             // flag for camera on/off
static GtkImage *GUIImage;          // pointer to video window
static GtkLabel *GUILabel;          // pointer to time display label
static Mat presentFrame;            // present frame to display

static gboolean update_vid_window (gpointer userdata) {
    get_present_image ( & presentFrame );
    //if (presentFrame.data == NULL)
    //    printf("null then\n");
    //else
    //    printf("not null\n");
    printf("frame size (%d, %d)\n", presentFrame.rows, presentFrame.cols);
    gtk_image_set_from_pixbuf (GUIImage,
                               gdk_pixbuf_new_from_data(presentFrame.data, GDK_COLORSPACE_RGB,
                                                        false, 8, presentFrame.cols, presentFrame.rows,
                                                        presentFrame.step, NULL, NULL));
    return G_SOURCE_REMOVE;
}

// update GUI time display
static gboolean update_GUI_time (gpointer userdata) {
    time_t rawtime;
    struct tm *info;
    char buffer[80];

    time( &rawtime );

    info = localtime( &rawtime );

    double timeInMilliSec;
	struct timeval curTime;
	gettimeofday ( &curTime, NULL );
    timeInMilliSec = (double) curTime.tv_usec * 1e-3 ; // Current time

    //printf("ms is %d.\n", (int)l_time_ms);
    sprintf(buffer, "%smillisec: %d", asctime(info), (int)timeInMilliSec);

    // asctime: Converts given calendar time std::tm to a textual representation of the following fixed 25-character form: Www Mmm dd hh:mm:ss yyyy\n

    gtk_label_set_text (GUILabel, buffer);
    return G_SOURCE_REMOVE;
}

static void* GUI_update_THREAD ( void *threadid ) {
    //g_print ("Hello World\n")
    printf("at the start of GUI_update_THREAD.\n");
    while (fThread) {
        g_main_context_invoke (NULL, update_GUI_time  , NULL);
        if (fCam) {
            g_main_context_invoke (NULL, update_vid_window, NULL);
        }
        my_sleep (16);
    }
    printf("at the end of GUI_update_THREAD.\n");
}


void GUI_master_activate ( GtkImage *vidWindow, GtkLabel *timeLabel ) {
    GUIImage = vidWindow;
    GUILabel = timeLabel;
    pthread_t GUI_update_thread;
    fThread = 1;
    pthread_create ( &GUI_update_thread, NULL, GUI_update_THREAD, NULL);  //start control loop thread

}

void GUI_master_deactivate(void) {
    fThread = 0;
    my_sleep (100);
    //usleep(1e5);
}

// start/stop camera stream
void on_toggle_camera_stream_toggled (GtkToggleButton *togglebutton, gpointer data) {
    int d = gtk_toggle_button_get_active (togglebutton);
    printf("toggle button value %d.\n", d);
    if (d) {
        camera_activate ();
    } else {
        camera_deactivate ();
    }
    my_sleep(1000);
    fCam = d;
}
