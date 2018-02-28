#include "vision.hpp"

static int fThread = 0;             // flag of whether or not thread is running
static FWcamera cam; // create a new instance of FWcamera
static Mat presentFrame = Mat(480,640,CV_8UC3);            // present captured frame
static Mat frameForDisplay = Mat(480,640,CV_8UC3);         // different with presentFrame by annotation/drawing
static int copyLock = 0;
static int fArena  = 0;                                 // 0: hide; 1: show digital arena
static int fSim = 0;                            // 0: real experiment; 1: simulation
static int thresholdPara = 65;

static Point robotPos, cargoPos;
static int fRobotPos=0, fCargoPos=0;                    // data lock

static float robotAngle = 0, cargoAngle = 0;                            // cargo orientation
static int fRobotAngle = 0, fCargoAngle = 0;

/* define the method for sorting contours */
static bool compareContourArea (vector<Point> contour1, vector<Point> contour2) {
    double i = fabs ( contourArea (contour1, false) );
    double j = fabs ( contourArea (contour2, false) );
    return (i < j);
}

/* class definition */
class Vision_Master {
    private:
        Mat binaryImg;          // binary image
        Mat grayImg;            // gray image

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        RotatedRect boundingRect1, boundingRect2;
    public:
        Mat colorImg;
        int nContours;
        vector<Point> sortedContour1, sortedContour2;
        void get_latest_frame   (void);
        void convert_to_binary  (void);
        void convert_to_color   (void);
        void find_contours      (void);
        void sort_contours      (void);
        void get_bounding_rect  (void);
        void update_robot_and_cargo_pos   (void);
        void draw_detection     (void);
        void update_robot_and_cargo_angle (void);
};

/* get the latest frame */
void Vision_Master :: get_latest_frame (void) {
    if ( fSim ) {                            // If for testing
        Mat whatever = imread("4.png", IMREAD_GRAYSCALE );
        whatever.copyTo ( grayImg );
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
    if (nContours >= 2)
        boundingRect2 = minAreaRect( Mat( sortedContour2 ) );
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
            cargoPos.x = boundingRect2.center.x;
            cargoPos.y = boundingRect2.center.y;
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
        Point2f rect_points[4];
        boundingRect2.points( rect_points );
        for (int i = 0; i < 4; i ++) {
            line( colorImg, rect_points[i], rect_points[(i+1)%4], Scalar(255,0,0), 1, 8 );
        }
    }
}

void Vision_Master :: update_robot_and_cargo_angle (void) {
    if ( nContours >= 2 ) {
        float tempAngle = boundingRect1.angle;          // get angle of rotated rect
        float width  = boundingRect1.size.width;
        float height = boundingRect1.size.height;
        int checkPt[8];
        printf("angle: %.3f, width: %.3f, height: %.3f\n", tempAngle, width, height);
        checkPt[0] = boundingRect1.center.x - 0.5 * width  * cosd(tempAngle);
        checkPt[1] = boundingRect1.center.y - 0.5 * height * sind(tempAngle);
        checkPt[2] = boundingRect1.center.x + 0.5 * height * sind(tempAngle);
        checkPt[3] = boundingRect1.center.y - 0.5 * height * cosd(tempAngle);
        checkPt[4] = boundingRect1.center.x + 0.5 * width  * cosd(tempAngle);
        checkPt[5] = boundingRect1.center.y + 0.5 * height * sind(tempAngle);
        checkPt[6] = boundingRect1.center.x - 0.5 * width  * sind(tempAngle);
        checkPt[7] = boundingRect1.center.y + 0.5 * height * cosd(tempAngle);
        circle(colorImg, Point(boundingRect1.center.x, boundingRect1.center.y), 10, Scalar(255,255,255));
        circle(colorImg, Point(checkPt[0], checkPt[1]), 5, Scalar(255,0,0));
        circle(colorImg, Point(checkPt[2], checkPt[3]), 5, Scalar(255,0,0));
        circle(colorImg, Point(checkPt[4], checkPt[5]), 5, Scalar(255,0,0));
        circle(colorImg, Point(checkPt[6], checkPt[7]), 5, Scalar(255,0,0));

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
    Vision_Master myVision;

    while ( fThread ) {
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
            draw_digital_arena ( &myVision.colorImg );

        while (copyLock);
        copyLock = 1;
        myVision.colorImg.copyTo(frameForDisplay);
        copyLock = 0;
        my_sleep(30);           // correspond to 30 Hz camera rate
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
