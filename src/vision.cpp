#include "vision.hpp"

static int fThread = 0;             // flag of whether or not thread is running
static FWcamera cam; // create a new instance of FWcamera
static Mat presentFrame;            // present captured frame
static int copyLock = 0;

static void* video_stream_THREAD ( void *threadid ) {
    printf("at the start of video_stream_THREAD.\n");
    /* use laptop webcamera instead of FWcamera */
    VideoCapture cap;

    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if ( !cap.open(0) )
        return 0;

    while (fThread) {
        while (copyLock);
        copyLock = 1;
        cap >> presentFrame;
        copyLock = 0;


        //img_m = Mat(480, 640, CV_8UC1, inImage);
          if( presentFrame.empty() ) break; // end of video stream
          //printf("frame size (%d, %d)\n", presentFrame.rows, presentFrame.cols);
          //imshow("this is you, smile! :)", frame);

          //if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC
          my_sleep(16);
    }
    // the camera will be closed automatically upon exit
    // cap.close();
    printf("at the end of video_stream_THREAD.\n");
}

void camera_activate (void) {

    /*
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
    */

    fThread = 1;
    pthread_t videoStreamThread;
    pthread_create( &videoStreamThread, NULL, video_stream_THREAD, NULL);  //start vision thread
}

void camera_deactivate (void) {
    fThread = 0;
}

void get_present_image ( Mat * container ) {
    while (copyLock);
    copyLock = 1;
    presentFrame.copyTo(*container);
    copyLock = 0;
}
