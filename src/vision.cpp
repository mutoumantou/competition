#include "vision.hpp"

static int fThread = 0;             // flag of whether or not thread is running
static FWcamera cam; // create a new instance of FWcamera
static Mat presentFrame = Mat(480,640,CV_8UC3);            // present captured frame
static Mat frameForDisplay = Mat(480,640,CV_8UC3);         // different with presentFrame by annotation/drawing
static int copyLock = 0;
static int fCamera = 0;                                 // 0: workstation, using FWcamera; 1: laptop, use webcamera
static int fArena  = 0;                                 // 0: hide; 1: show digital arena

static int thresholdPara = 65;

static Point robotPos, cargoPos;
static int fRobotPos=0, fCargoPos=0;                    // data lock

static float cargoAngle = 0;                            // cargo orientation
static int fCargoAngle = 0;

/* draw digital arena on image */
static void draw_digital_arena ( Mat * data ) {
    /* inner top-left corner @ (7, 61), 5.6 um/pixel */
    rectangle ( *data, Point(  7, 61), Point(632,418), Scalar(255,0,0) );     // length: 3.5 mm (625 pixels), height: 2 mm (357 pixel)
    rectangle ( *data, Point(311, 61), Point(320,106), Scalar(255,0,0) );     // length: 50 um; height: 250 um
    rectangle ( *data, Point(311,177), Point(320,240), Scalar(255,0,0) );     // length: 50 um; height: 350 um
    line      ( *data, Point(  0, 52), Point(639, 52), Scalar(255,0,0) );
    line      ( *data, Point(  0,427), Point(639,427), Scalar(255,0,0) );
}

static void* video_stream_THREAD ( void *threadid ) {
    printf("at the start of video_stream_THREAD.\n");
    /* use laptop webcamera instead of FWcamera */

    while ( fThread ) {
        Mat img_m_color, binaryImg;
        Mat img_m_gray;

        if ( fCamera ) {
            Mat tempFrame = Mat(480,640,CV_8UC3);
            /*
            VideoCapture cap;               // may cause crash problem of GTK2.0 conflick with GTK3.0 on workstation computer

            // open the default camera, use something different from 0 otherwise;
            // Check VideoCapture documentation.
            if ( !cap.open(0) )
                return 0;

            cap >> presentFrame;
            */
            presentFrame.copyTo(tempFrame);



                //printf("frame size (%d, %d), depth %d channel %d elesize %d type %d\n", tempFrame.rows, tempFrame.cols, tempFrame.depth(), tempFrame.channels(), tempFrame.elemSize(), tempFrame.type());
            for (int i = 0; i < presentFrame.rows; i ++) {
                for (int j = 0; j < presentFrame.cols; j ++) {
                    tempFrame.at<unsigned char>(i,j*3) = presentFrame.at<unsigned char>(i,j*3+2);
                    tempFrame.at<unsigned char>(i,j*3+2) = presentFrame.at<unsigned char>(i,j*3);
                }
            }

            tempFrame.copyTo(presentFrame);
            cvtColor ( presentFrame, img_m_gray, CV_RGB2GRAY );

            //Mat img_m_gray = imread("0.png", 0 );
        } else {
            unsigned char *inImage;
            //printf("before grab\n");
            inImage = cam.grabAframe();
            //printf("after grab\n");
            if(inImage == NULL)	{
                g_print("Error in firewire stream! Reattempting...\n");
                my_sleep(1); 																										// I don't know what the wait delay should be
            }
            Mat img_m = Mat(480, 640, CV_8UC1, inImage);
            img_m.copyTo (img_m_gray);


            //cvtColor(img_m, img_m_color, CV_GRAY2BGR);
        }
        //printf("marker 1\n");
        /* object detection */
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        //dilate( img_m, img_m, Mat(), Point(-1, -1), 10, 1, 1);
	    //erode( img_m, img_m, Mat(), Point(-1, -1), 15, 1, 1);

        //blur      ( img_m, binaryImg, Size(4,4) );       // blur image to remove small blips etc
        threshold    ( img_m_gray, binaryImg, thresholdPara, 255, THRESH_BINARY_INV );
        //cvtColor ( binaryImg, img_m_color, CV_GRAY2BGR );
        cvtColor ( img_m_gray, img_m_color, CV_GRAY2BGR );
        findContours ( binaryImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) ); //find contours

        if (contours.size() < 1)
            printf("contours size is %d.\n", (int)contours.size());
        else {
    		//first, find the largest contour    ...   Contour classification!
    		int largestArea = -1;
            int secondArea = -1;

            int iLargestContour = -1;               // index of largest contour
            int i2ndContour = -1;

    		for(int i = 0; i < contours.size(); i++ ) {
    			int a = contourArea(contours[i], false);           //  Find the area of contour
    			if ( a > largestArea ) {                           // if current contour is bigger ...
                    secondArea = largestArea;
                    i2ndContour = iLargestContour;
    				largestArea = a;
    				iLargestContour = i;                 //Store the index of largest contour
    			} else if ( a > secondArea ) {
                    secondArea = a;
                    i2ndContour = i;
                }
    		}

            RotatedRect rotatedBoundingRect1, rotatedBoundingRect2;
            rotatedBoundingRect1 = minAreaRect( Mat( contours [iLargestContour] ) );
            if (i2ndContour == -1) {
                while (fCargoPos);
                fCargoPos = 1;
                cargoPos.x = 0;
                cargoPos.y = 0;
                fCargoPos = 0;
            } else {
                rotatedBoundingRect2 = minAreaRect( Mat( contours [i2ndContour] ) );
                /* get center point position of cargo */
                while (fCargoPos);
                fCargoPos = 1;
                cargoPos.x = rotatedBoundingRect2.center.x;
                cargoPos.y = rotatedBoundingRect2.center.y;
                fCargoPos = 0;
                Point2f rect_points2[4];
                rotatedBoundingRect2.points( rect_points2 );
                for (int j = 0; j < 4; j ++) {
                    line( img_m_color, rect_points2[j], rect_points2[(j+1)%4], Scalar(255,0,0), 1, 8 );
                }
                float tempAngle = 0;
                if (rotatedBoundingRect2.size.width < rotatedBoundingRect2.size.height) {
                    tempAngle = -1 * rotatedBoundingRect2.angle - 90;
                } else
                    tempAngle = -1 * rotatedBoundingRect2.angle;
                //printf("angle %.3f\n", tempAngle);
                while (fCargoAngle);
                fCargoAngle = 1;
                cargoAngle = tempAngle;
                fCargoAngle = 0;
            }

            //printf("width: %.3f, height: %.3f, angle: %.3f\n", rotatedBoundingRect2.size.width, rotatedBoundingRect2.size.height, rotatedBoundingRect2.angle);
            /* get center point position of robot */
            while (fRobotPos);
            fRobotPos = 1;
            robotPos.x = rotatedBoundingRect1.center.x;
            robotPos.y = rotatedBoundingRect1.center.y;
            fRobotPos = 0;

            float angle;
            angle = rotatedBoundingRect1.angle;

            Point2f rect_points1[4];
            rotatedBoundingRect1.points( rect_points1 );

            /* draw detected rect. */
            for (int j = 0; j < 4; j ++) {
                line( img_m_color, rect_points1[j], rect_points1[(j+1)%4], Scalar(255,0,0), 1, 8 );
            }

    	}

        /* draw digital arena if needed */
        if ( fArena )
            draw_digital_arena ( &img_m_color );

        while (copyLock);
        copyLock = 1;
        img_m_color.copyTo(frameForDisplay);
        copyLock = 0;
        my_sleep(30);           // correspond to 30 Hz camera rate
        //my_sleep(1000);
    }

    //img_m = Mat(480, 640, CV_8UC1, inImage);
    //if( presentFrame.empty() ) break; // end of video stream
      //printf("frame size (%d, %d)\n", presentFrame.rows, presentFrame.cols);
      //imshow("this is you, smile! :)", frame);
      //if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC
    //printf("frame size (%d, %d), depth %d channel %d elesize %d type %d\n", tempFrame.rows, tempFrame.cols, tempFrame.depth(), tempFrame.channels(), tempFrame.elemSize(), tempFrame.type());

    // the camera will be closed automatically upon exit
    // cap.close();
    if (!fCamera) {
        cam.stopGrabbingVideo();
    	my_sleep(100);
    	cam.deinitialize();
        cam.deinitialize();
    }

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

/* show/hide digital arena */
void on_toggle_arena_toggled (GtkToggleButton *togglebutton, gpointer data) {
    fArena = gtk_toggle_button_get_active (togglebutton);
}

void on_spin_binaryThreshold_changed (GtkEditable *editable, gpointer user_data) {
    thresholdPara = gtk_spin_button_get_value ( GTK_SPIN_BUTTON(editable) );
}

/* return current position information of robot and cargo to controller */
void return_center_pt_info ( Point *robot, Point *cargo, float *angle) {
    while (fRobotPos);
    fRobotPos = 1;
    (*robot).x = robotPos.x;
    (*robot).y = 480 - robotPos.y;
    fRobotPos = 0;

    while (fCargoPos);
    fCargoPos = 1;
    (*cargo).x = cargoPos.x;
    (*cargo).y = 480 - cargoPos.y;
    fCargoPos = 0;

    while (fCargoAngle);
    fCargoAngle = 1;
    (*angle) = cargoAngle;
    fCargoAngle = 0;
}
