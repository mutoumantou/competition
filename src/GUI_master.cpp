#include "GUI_master.hpp"

static int fThread = 0;

static void* GUI_update_THREAD ( void *threadid ) {
    //g_print ("Hello World\n")
    printf("at the start of GUI_update_THREAD.\n");
    while (fThread);
    printf("at the end of GUI_update_THREAD.\n");
}


void GUI_master_activate ( void ) {
    pthread_t GUI_update_thread;
    fThread = 1;
    pthread_create ( &GUI_update_thread, NULL, GUI_update_THREAD, NULL);  //start control loop thread

}

void GUI_master_deactivate(void) {
    fThread = 0;
    struct timespec req, rem;
    nanosleep(&req, &rem);
    //usleep(1e5);
}
