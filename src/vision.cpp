#include "vision.hpp"

static FWcamera cam; // create a new instance of FWcamera

static void* video_stream_THREAD ( void *threadid ) {
    printf("at the start of video_stream_THREAD.\n");
    printf("at the end of video_stream_THREAD.\n");
}

void camera_activate (void) {
    pthread_t videoStreamThread;

    if( !cam.initialize() ) {
        g_print("Error: camera could not be found in initVision().\n");
        return;
    }
    my_sleep(100);
    if( !cam.startGrabbingVideo() )	{
        g_print("Error: camera could not grab in startGrabbingVideo().\n");
        return;
    }
    my_sleep(100);
    pthread_create( &videoStreamThread, NULL, video_stream_THREAD, NULL);  //start vision thread
}

void camera_deactivate (void) {

}
