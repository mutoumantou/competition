#include "vision.hpp"

static int fThread = 0;             // flag of whether or not thread is running
static FWcamera cam; // create a new instance of FWcamera
static Mat presentFrame = Mat(480,640,CV_8UC3);            // present captured frame
static Mat frameForDisplay = Mat(480,640,CV_8UC3);         // different with presentFrame by annotation/drawing
static int copyLock = 0;
static int fArena  = 0;                                 // 0: hide; 1: show digital arena
static int fSim = 0;                            // 0: real experiment; 1: simulation
static int thresholdPara = 65;
static int iCargo = 0;                          // cargo type. 0: circle; 1: rectangle; 2: triangle
static Point robotPos, cargoPos;
static int fRobotPos=0, fCargoPos=0;                    // data lock

static float robotAngle = 0, cargoAngle = 0;                            // cargo orientation
static int fRobotAngle = 0, fCargoAngle = 0;

/* define the method for sorting contours */

static bool compareContourArea (std::vector<Point> contour1, std::vector<Point> contour2) {
    double i = fabs ( contourArea (contour1, false) );
    double j = fabs ( contourArea (contour2, false) );
    return (i < j);
}


/* class definition */
class Vision_Master {
    private:
        Mat binaryImg;          // binary image
        Mat grayImg;            // gray image

        std::vector<std::vector<Point> > contours;
        std::vector<Vec4i> hierarchy;
        RotatedRect boundingRect1, boundingRect2;
        /// parameter used when cargo is circle
        Point2f circleCenter;
        float circleRadius;
    public:
        Mat colorImg;
        int nContours;
        std::vector<Point> sortedContour1, sortedContour2;
        void get_latest_frame   (void);
        void convert_to_binary  (void);
        void convert_to_color   (void);
        void find_contours      (void);
        void sort_contours      (void);
        void get_bounding_rect  (void);
        void update_robot_and_cargo_pos   (void);
        void draw_detection     (void);
        void update_robot_and_cargo_angle (void);
        void draw_digital_arena (void);
};

/* get the latest frame */
void Vision_Master :: get_latest_frame (void) {

    if ( fSim ) {                            // If for testing
        //Mat whatever = imread("4.png", IMREAD_GRAYSCALE );        // (on workstation) cause GTK+ 2.x symbols detected. Using GTK+ 2.x and GTK+ 3 in the same process is not supported

        //whatever.copyTo ( grayImg );
    } else {                                 // if for real experiments
        unsigned char *inImage;
        //printf("before grab\n");
        inImage = cam.grabAframe();
        //printf("after grab\n");
        if(inImage == NULL)	{
            g_print("Error in firewire stream! Reattempting...\n");
            my_sleep(1); 																										// I don't know what the wait delay should be
        }
        Mat img_m = Mat(480, 640, CV_8UC1, inImage);
        img_m.copyTo (grayImg);
    }

}

void Vision_Master :: convert_to_binary (void) {
    threshold ( grayImg, binaryImg, thresholdPara, 255, THRESH_BINARY_INV );
}

void Vision_Master :: convert_to_color (void) {
    cvtColor ( grayImg, colorImg, CV_GRAY2BGR );
}

void Vision_Master :: find_contours (void) {

    findContours ( binaryImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    nContours = contours.size();
    if (nContours < 1)
        printf("Error: find no contours.\n");

}

void Vision_Master :: sort_contours (void) {

    sort ( contours.begin(), contours.end(), compareContourArea );
    if (nContours >= 1)
        sortedContour1 = contours[nContours-1];
    if (nContours >= 2)
        sortedContour2 = contours[nContours-2];

}

void Vision_Master :: get_bounding_rect (void) {
    if (nContours >= 1)
        boundingRect1 = minAreaRect( Mat( sortedContour1 ) );
    if (nContours >= 2) {
        switch (iCargo) {
            case 0:
                minEnclosingCircle ( Mat( sortedContour2 ), circleCenter, circleRadius);
                break;
            case 1:
            case 2:
                boundingRect2 = minAreaRect( Mat( sortedContour2 ) );
        }
    }

}

void Vision_Master :: update_robot_and_cargo_pos (void) {
    if (nContours >= 1) {
        while (fRobotPos);
        fRobotPos = 1;
        robotPos.x = boundingRect1.center.x;
        robotPos.y = boundingRect1.center.y;
        fRobotPos = 0;

        if ( nContours < 2 ) {
            while (fCargoPos);
            fCargoPos = 1;
            cargoPos.x = 0;
            cargoPos.y = 0;
            fCargoPos = 0;
        } else {
            while (fCargoPos);
            fCargoPos = 1;
            if (iCargo == 0) {                  // when cargo is a circle
                cargoPos.x = circleCenter.x;
                cargoPos.y = circleCenter.y;
            } else {
                cargoPos.x = boundingRect2.center.x;
                cargoPos.y = boundingRect2.center.y;
            }
            fCargoPos = 0;
        }
    }

}

void Vision_Master :: draw_detection (void) {
    if (nContours >= 1) {
        Point2f rect_points[4];
        boundingRect1.points( rect_points );
        for (int i = 0; i < 4; i ++) {
            line( colorImg, rect_points[i], rect_points[(i+1)%4], Scalar(255,0,0), 1, 8 );
        }
    }
    if (nContours >= 2) {
        if (iCargo == 0) {
            circle ( colorImg, circleCenter, circleRadius, Scalar(255,255,255));
        } else {
            Point2f rect_points[4];
            boundingRect2.points( rect_points );
            for (int i = 0; i < 4; i ++) {
                line( colorImg, rect_points[i], rect_points[(i+1)%4], Scalar(255,0,0), 1, 8 );
            }
        }
    }
}

void Vision_Master :: update_robot_and_cargo_angle (void) {
    if ( nContours >= 2 ) {
        float tempAngle = boundingRect1.angle;          // get angle of rotated rect
        float width  = boundingRect1.size.width;
        float height = boundingRect1.size.height;
        int checkPt[8];
        //printf("angle: %.3f, width: %.3f, height: %.3f\n", tempAngle, width, height);
        checkPt[0] = boundingRect1.center.x - 0.4 * width  * cosd(tempAngle);
        checkPt[1] = boundingRect1.center.y - 0.4 * height * sind(tempAngle);
        checkPt[2] = boundingRect1.center.x + 0.4 * height * sind(tempAngle);
        checkPt[3] = boundingRect1.center.y - 0.4 * height * cosd(tempAngle);
        checkPt[4] = boundingRect1.center.x + 0.4 * width  * cosd(tempAngle);
        checkPt[5] = boundingRect1.center.y + 0.4 * height * sind(tempAngle);
        checkPt[6] = boundingRect1.center.x - 0.4 * width  * sind(tempAngle);
        checkPt[7] = boundingRect1.center.y + 0.4 * height * cosd(tempAngle);
        //printf("binary image channel %d depth %d\n", binaryImg.channels(), binaryImg.depth());
        //printf("pt1 %d pt2 %d pt3 %d pt4 %d\n", binaryImg.at<unsigned char>(checkPt[1],checkPt[0]),
        //                                        binaryImg.at<unsigned char>(checkPt[3],checkPt[2]),
        //                                        binaryImg.at<unsigned char>(checkPt[5],checkPt[4]),
        //                                        binaryImg.at<unsigned char>(checkPt[7],checkPt[6]));

        int iDir = -1;
        if (binaryImg.at<unsigned char>(checkPt[1],checkPt[0]) == 0)
            iDir = 0;
        else if (binaryImg.at<unsigned char>(checkPt[3],checkPt[2]) == 0)
            iDir = 1;
        else if (binaryImg.at<unsigned char>(checkPt[5],checkPt[4]) == 0)
            iDir = 2;
        else if (binaryImg.at<unsigned char>(checkPt[7],checkPt[6]) == 0)
            iDir = 3;

        circle(colorImg, Point(boundingRect1.center.x, boundingRect1.center.y), 10, Scalar(255,255,255));
        //circle(colorImg, Point(checkPt[0], checkPt[1]), 5, Scalar(255,0,0));
        //circle(colorImg, Point(checkPt[2], checkPt[3]), 5, Scalar(255,0,0));
        //circle(colorImg, Point(checkPt[4], checkPt[5]), 5, Scalar(255,0,0));
        //circle(colorImg, Point(checkPt[6], checkPt[7]), 5, Scalar(255,0,0));
        circle(colorImg, Point(checkPt[iDir*2], checkPt[iDir*2+1]), 5, Scalar(255,0,0));

        while (fRobotAngle);
        fRobotAngle = 1;
        robotAngle = tempAngle;
        fRobotAngle = 0;

        tempAngle = 0;
        if (boundingRect2.size.width < boundingRect2.size.height) {
            tempAngle = -1 * boundingRect2.angle - 90;
        } else
            tempAngle = -1 * boundingRect2.angle;
        while (fCargoAngle);
        fCargoAngle = 1;
        cargoAngle = tempAngle;
        fCargoAngle = 0;
    }
}

/* draw digital arena on image */
void Vision_Master :: draw_digital_arena ( void ) {
    /* inner top-left corner @ (7, 61), 5.6 um/pixel */
    rectangle ( colorImg, Point(  7, 61), Point(632,418), Scalar(255,0,0) );     // length: 3.5 mm (625 pixels), height: 2 mm (357 pixel)
    rectangle ( colorImg, Point(311, 61), Point(320,106), Scalar(255,0,0) );     // length: 50 um; height: 250 um
    rectangle ( colorImg, Point(311,177), Point(320,240), Scalar(255,0,0) );     // length: 50 um; height: 350 um
    line      ( colorImg, Point(  0, 52), Point(639, 52), Scalar(255,0,0) );
    line      ( colorImg, Point(  0,427), Point(639,427), Scalar(255,0,0) );

    switch (iCargo) {
        case 0:
            //circle    ( colorImg, Point(7+134+22, 61+116), 22, Scalar(255,0,0));
            circle    ( colorImg, Point(163, 177), 22, Scalar(255,0,0));
            break;
        case 1:
            rectangle ( colorImg, Point(7+89,61+116), Point(7+89+63,61+116+36), Scalar(255,0,0) );
            break;
        case 2:
            vector<Point> contour;
            contour.push_back (Point(7+89,61+116+36));
            contour.push_back (Point(7+89+63,61+116));
            contour.push_back (Point(7+89+63,61+116+36));

            const Point * pts = (const Point *) Mat(contour).data;
            int nPts = Mat(contour).rows;
            //Point myPt[3] = { Point(7+89,61+116+36),Point(7+89+63,61+116),Point(7+89+63,61+116+36) };
            //int nVortex[3] = {0, 1, 2};
            polylines ( colorImg, &pts, &nPts, 1, true, Scalar(255,0,0));
            break;
    }

}

static void* video_stream_THREAD ( void *threadid ) {
    printf("at the start of video_stream_THREAD.\n");

    Vision_Master myVision;

    //int i = 0;

    while ( fThread ) {
        //printf("earlier in loop %d ", i);
        myVision.get_latest_frame();
        myVision.convert_to_binary();
        myVision.convert_to_color();

        myVision.find_contours ();
        myVision.sort_contours ();


        myVision.get_bounding_rect ();

        myVision.update_robot_and_cargo_pos ();
        myVision.draw_detection ();
        myVision.update_robot_and_cargo_angle ();

        /* draw digital arena if needed */

        if ( fArena )
            myVision.draw_digital_arena ();

        while (copyLock);
        copyLock = 1;
        myVision.colorImg.copyTo(frameForDisplay);
        copyLock = 0;

        my_sleep(30);           // correspond to 30 Hz camera rate
        //printf("later in loop %d\n", i++);
    }

    if (!fSim) {
        cam.stopGrabbingVideo();
    	my_sleep(100);
    	cam.deinitialize();
        cam.deinitialize();
    }

    printf("at the end of video_stream_THREAD.\n");
}

/* activate the video capture and processing thread */
void camera_activate (void) {
    if ( !fSim ) {
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

void on_cB_simulation_toggled (GtkToggleButton *togglebutton, gpointer data) {
    fSim = gtk_toggle_button_get_active (togglebutton);
}

void on_rB_1_toggled (GtkToggleButton *togglebutton, gpointer data) {
    int d = gtk_toggle_button_get_active (togglebutton);
    if (d == 1) {
        iCargo = 0;
        printf("cargo selected to circle\n");
    }

}
void on_rB_2_toggled (GtkToggleButton *togglebutton, gpointer data) {
    int d = gtk_toggle_button_get_active (togglebutton);
    if (d == 1) {
        iCargo = 1;
        printf("cargo selected to rectangle\n");
    }
}
void on_rB_3_toggled (GtkToggleButton *togglebutton, gpointer data) {
    int d = gtk_toggle_button_get_active (togglebutton);
    if (d == 1) {
        iCargo = 2;
        printf("cargo selected to triangle\n");
    }
}

int get_cargo_type (void) {
    return iCargo;
}
