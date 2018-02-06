#include "actuation.hpp"

static int fThread = 0;             // flag of thread running
static int directionCode = -1;      // -1: neutral; 0: +x; 1: +y; 2: -x; 3: -y

/* thread of actuation */
static void* actuation_THREAD ( void *threadid ) {
    printf("at the start of actuation_THREAD.\n");
    while (fThread) {
        my_sleep(1000);
    }
    printf("at the end of actuation_THREAD.\n");
}

/* start or stop the actuation thread */
void on_tB_actuation_toggled (GtkToggleButton *togglebutton, gpointer data) {
    int d = gtk_toggle_button_get_active (togglebutton);

    if (d) {
        fThread = 1;
        pthread_t actuationThread;
        pthread_create( &actuationThread, NULL, actuation_THREAD, NULL);  //start vision thread
    } else {
        fThread = 0;
    }
}

void set_directionCode (int keycode) {
    directionCode = keycode;
    switch (directionCode) {
        case -1: printf("neutral\n"); break;
        case  0: printf("right\n"); break;
        case  1: printf("up\n"); break;
        case  2: printf("left\n"); break;
        case  3: printf("down\n"); break;
    }
}
