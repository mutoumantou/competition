#include "GUI_master.hpp"

static int fThread = 0;
static GtkImage *GUIImage;
static GtkLabel *GUILabel;

static void my_sleep (unsigned msec) {
    struct timespec req, rem;
    int err;
    req.tv_sec = msec / 1000;
    req.tv_nsec = (msec % 1000) * 1000000;
    while ((req.tv_sec != 0) || (req.tv_nsec != 0)) {
        if (nanosleep(&req, &rem) == 0)
            break;
        err = errno;
        // Interrupted; continue
        if (err == EINTR) {
            req.tv_sec = rem.tv_sec;
            req.tv_nsec = rem.tv_nsec;
        }
        // Unhandleable error (EFAULT (bad pointer), EINVAL (bad timeval in tv_nsec), or ENOSYS (function not supported))
        break;
    }
}

// update GUI time display
static gboolean update_GUI_time (gpointer userdata) {
    time_t rawtime;
    struct tm *info;
    char buffer[80];

    time( &rawtime );

    info = localtime( &rawtime );
    //printf("Current local time and date: %s", asctime(info));

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
        g_main_context_invoke (NULL, update_GUI_time, NULL);

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
