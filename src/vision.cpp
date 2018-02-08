#include "vision.hpp"

static int fThread = 0;             // flag of whether or not thread is running
static FWcamera cam; // create a new instance of FWcamera
static Mat presentFrame = Mat(480,640,CV_8UC3);            // present captured frame
static Mat frameForDisplay = Mat(480,640,CV_8UC3);         // different with presentFrame by annotation/drawing
static int copyLock = 0;
static int fCamera = 1;                                 // 0: workstation, using FWcamera; 1: laptop, use webcamera

static void* video_stream_THREAD ( void *threadid ) {
    printf("at the start of video_stream_THREAD.\n");
    /* use laptop webcamera instead of FWcamera */
    if ( fCamera ) {

        VideoCapture cap;               // may cause crash problem of GTK2.0 conflick with GTK3.0 on workstation computer
        Mat tempFrame = Mat(480,640,CV_8UC3);
        // open the default camera, use something different from 0 otherwise;
        // Check VideoCapture documentation.
        if ( !cap.open(0) )
            return 0;
        while ( fThread ) {
            cap >> presentFrame;
            presentFrame.copyTo(tempFrame);
            //printf("frame size (%d, %d), depth %d channel %d elesize %d type %d\n", tempFrame.rows, tempFrame.cols, tempFrame.depth(), tempFrame.channels(), tempFrame.elemSize(), tempFrame.type());
            for (int i = 0; i < presentFrame.rows; i ++) {
                for (int j = 0; j < presentFrame.cols; j ++) {
                    tempFrame.at<unsigned char>(i,j*3) = presentFrame.at<unsigned char>(i,j*3+2);
                    tempFrame.at<unsigned char>(i,j*3+2) = presentFrame.at<unsigned char>(i,j*3);
                    //tempFrame.at<unsigned int>(i,j) = ( ( presentFrame.at<unsigned int>(i,j) >> 16 ) & 0x000000FF ) |
                    //                                  ( ( presentFrame.at<unsigned int>(i,j) << 16 ) & 0x00FF0000 ) |
                    //                                  ( presentFrame.at<unsigned int>(i,j) & 0xFF00FF00 );
                }
            }
            //printf("take a look hex %#x\n", tempFrame.at<unsigned int>(0,0));
            //my_sleep(10000);

            while (copyLock);
            copyLock = 1;
            tempFrame.copyTo(frameForDisplay);
            copyLock = 0;


            //img_m = Mat(480, 640, CV_8UC1, inImage);
              if( presentFrame.empty() ) break; // end of video stream
              //printf("frame size (%d, %d)\n", presentFrame.rows, presentFrame.cols);
              //imshow("this is you, smile! :)", frame);

              //if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC
            my_sleep(30);
        }
    } else {
        unsigned char *inImage;

        while (fThread) {
            //printf("before grab\n");
            inImage = cam.grabAframe();
            //printf("after grab\n");
            if(inImage == NULL)	{
                g_print("Error in firewire stream! Reattempting...\n");
                my_sleep(1); 																										// I don't know what the wait delay should be
            }
            Mat img_m = Mat(480, 640, CV_8UC1, inImage);
            Mat img_m_color;

            cvtColor(img_m, img_m_color, CV_GRAY2BGR);

            while (copyLock);
            copyLock = 1;
            img_m_color.copyTo(frameForDisplay);
            copyLock = 0;
            my_sleep(30);           // correspond to 30 Hz camera rate
        }
    }

    //printf("frame size (%d, %d), depth %d channel %d elesize %d type %d\n", tempFrame.rows, tempFrame.cols, tempFrame.depth(), tempFrame.channels(), tempFrame.elemSize(), tempFrame.type());

    // the camera will be closed automatically upon exit
    // cap.close();
    printf("at the end of video_stream_THREAD.\n");
}

void camera_activate (void) {
    if ( !fCamera ) {
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
    }

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
    frameForDisplay.copyTo(*container);
    copyLock = 0;
}
