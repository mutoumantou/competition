#include "actuation.hpp"

static int fThread = 0;             // flag of thread running
//static int fGradient = 0;           // 0: use uniform field; 1: use gradient field
static int directionCode = 0;      // -1: neutral; 0: +x; 1: +y; 2: -x; 3: -y
static int fKey = 0;                // if a key has been pressed, request reviewing direction

/* thread of actuation */
static void* actuation_THREAD ( void *threadid ) {
    printf("at the start of actuation_THREAD.\n");

    /* variable definition */
    float movingAngle = 0.0;              // desired moving angle of robot in XY plane degrees
    double presentTime = 0;

    MMC_Controller ctr;                 // get an instance of the controller
    ctr.get_latest_pos (0);             // init. run to get robot and cargo pos.

    float dis2 = 0.0;                   // distance from robot to destination
    float contactPos[2] = {0,0};        // position when contact happens
    double contactTime = 0.0;            // time when contact happens; has to be "double", otherwise will have error when computing with presentTime
    int nSwitch = 1;                    // no. of switch to push cargo from stuck; start from 1, so that robot moves a distance before oscillation

    int wayPoint_x[3] = {500, 160, 163};
    int wayPoint_y[3] = {150, 150, 480-177};
    int iWaypoint = 0;

    int iCargo = get_cargo_type ();         // get cargo type specified on GUI, vision.cpp

    /* start low-level coil thread: output sawtooth wave */
    start_coil_thread ();                   // coil.cpp

    while (fThread) {
        presentTime = get_present_time ();
        //printf("present time: %.3f.\n", presentTime);

        int rst = ctr.get_latest_pos (1);           // get current position info. of robot and cargo
        //printf("robot (%d, %d), cargo (%d, %d)\n", robotPos.x, robotPos.y, cargoPos.x, cargoPos.y);

        switch (ctr.state) {
            /* Step 0: move robot to cargo */
            case 0:
                if (rst) {               // if cargo detection is valid ...
                    if (iCargo != 0) {        // if cargo not circle
                        ctr.update_goal_info_using_cargo_pos();
                        change_moving_angle ( ctr.angle );

                        if (ctr.dis < 20) {                   // if reaching the goal
                            ctr.state = 1;
                            printf("reach state 1.\n");
                        }
                    } else {
                        ctr.state = 1;
                        printf("circle cargo skip state 1.\n");
                    }
                } else {
                    // in this state, just igore abnormal detection
                }
                break;
            /* Step: align robot with cargo's orientation */
            case 1:
                pause_coil_output (1);                       // coil.cpp
                ctr.set_cargo_as_goal ();
                change_moving_angle ( ctr.angle );
                printf("stage 1: change move anlge %.3f\n", ctr.angle);
                my_sleep (1000);                        // wait for robot to rotate

                /* check the alignment between robot and cargo */
                rst = ctr.get_latest_pos (1);                   // get robot and cargo position after rotation
                ctr.calc_angle_difference_to_desired_line ();   // see how much misalignment
                printf("stage 1: call calc. angle difference, result %.3f\n", ctr.angleDiff);
                pause_coil_output (0);
                if ( abs(ctr.angleDiff) >= 10 ) {
                    int angleSign = 0;
                    float initAngleDiff = ctr.angleDiff;
                    if (ctr.angleDiff > 0) {
                        change_moving_angle ( ctr.angle - 2 * ctr.angleDiff);
                        angleSign = 1;
                    } else {
                        change_moving_angle ( ctr.angle - 2 * ctr.angleDiff);
                        angleSign = -1;
                    }
                    //printf("stage 1: need re-adjustment. set angle to %.3f\n", ctr.angle - ctr.angleDiff);
                    change_moving_dir (1);              // coil.cpp; moving backwards
                    while ( fabs(ctr.angleDiff) >= ( 0.5 * initAngleDiff ) ) {
                        rst = ctr.get_latest_pos (1);
                        ctr.calc_angle_difference_to_desired_line ();
                        printf("stage 1: inner loop: call calc. angle difference, result %.3f\n", ctr.angleDiff);
                        change_moving_angle ( ctr.angle - 2 * ctr.angleDiff);
                        my_sleep(10);
                    }
                    change_moving_angle ( ctr.angle + 2 * ctr.angleDiff );
                    change_moving_dir (0);              // coil.cpp; moving forwards

                    while ( fabs(ctr.angleDiff) >= 1 ) {
                        rst = ctr.get_latest_pos (1);
                        ctr.calc_angle_difference_to_desired_line ();
                        printf("stage 1: inner loop: call calc. angle difference, result %.3f\n", ctr.angleDiff);
                        change_moving_angle ( ctr.angle + 2 * ctr.angleDiff);
                        my_sleep(10);
                    }
                    change_moving_angle ( ctr.angle );

                } else
                    printf("do not need re-adjustment.\n");
                //my_sleep (1000);                                // wait for user to review terminal output

                ctr.state = 2;
                printf("reach state 2.\n");
                break;
            /* Step: contact cargo */
            case 2:
                if ( ! ctr.fContact ) {       // if contact has not happended ...
                    printf("robot has NOT touched cargo.\n");
                    if ( rst ) {                            // if cargo detection is valid ...
                        printf("cargo detection is valid.\n");
                        ctr.set_cargo_as_goal ();
                        change_moving_angle ( ctr.angle );
                        ctr.calc_angle_difference_to_desired_line ();   // see how much misalignment
                        if (abs(ctr.angleDiff) >= 10) {
                            int angleSign = 0;
                            float initAngleDiff = ctr.angleDiff;
                            if (ctr.angleDiff > 0) {
                                change_moving_angle ( ctr.angle - 2 * ctr.angleDiff);
                                angleSign = 1;
                            } else {
                                change_moving_angle ( ctr.angle - 2 * ctr.angleDiff);
                                angleSign = -1;
                            }
                            //printf("stage 1: need re-adjustment. set angle to %.3f\n", ctr.angle - ctr.angleDiff);
                            change_moving_dir (1);              // coil.cpp; moving backwards
                            while ( fabs(ctr.angleDiff) >= ( 0.5 * initAngleDiff ) ) {
                                rst = ctr.get_latest_pos (1);
                                ctr.calc_angle_difference_to_desired_line ();
                                printf("stage 1: inner loop: call calc. angle difference, result %.3f\n", ctr.angleDiff);
                                change_moving_angle ( ctr.angle - 2 * ctr.angleDiff);
                                my_sleep(10);
                            }
                            change_moving_angle ( ctr.angle + 2 * ctr.angleDiff );
                            change_moving_dir (0);              // coil.cpp; moving forwards

                            while ( fabs(ctr.angleDiff) >= 1 ) {
                                rst = ctr.get_latest_pos (1);
                                ctr.calc_angle_difference_to_desired_line ();
                                printf("stage 1: inner loop: call calc. angle difference, result %.3f\n", ctr.angleDiff);
                                change_moving_angle ( ctr.angle + 2 * ctr.angleDiff);
                                my_sleep(10);
                            }
                            change_moving_angle ( ctr.angle );
                        }
                    } else {                        // if cargo detection is not valid ...
                        printf("cargo detection is invalid.\n");
                        rst = ctr.check_contact (iCargo);     // check if contact happened
                        if (rst) {                      // if robot has contacted cargo ...
                            contactPos[0] = ctr.robot.x;  // record the robot position when contact happens
                            contactPos[1] = ctr.robot.y;
                            contactTime = presentTime;
                            printf("robot has touched cargo at time %.3f.\n", contactTime);
                        }                               // otherwise just igore this detection
                    }
                } else {                      // if contact has happened
                    float tempDis = sqrt  (   pow ( ctr.robot.x - contactPos[0], 2 )
                                            + pow ( ctr.robot.y - contactPos[1], 2 ) ); // dis. between current pos. and contact pos.
                    printf("dis. between current pos. and contact pos. %.3f.\n", tempDis);
                    if ( (tempDis > 30) && (ctr.robot_away_from_init_pos()) ) {
                        ctr.state = 3;
                        printf("reach state 3.\n");
                    } else {
                        if ( ( (int)(presentTime - contactTime) ) > nSwitch * 2 ) {
                            nSwitch ++;
                            printf("nSwitch: %d, present time: %.3f, contact time: %.3f.\n", nSwitch, presentTime, contactTime);
                            if ( nSwitch / 2 % 2 == 0 )
                                change_moving_angle ( ctr.angle + 15 );
                            else
                                change_moving_angle ( ctr.angle - 15 );     // change pushing angle to overcome friction
                        }
                    }
                }
                break;
            /* Step: pass waypoints */
            case 3:
                if (iWaypoint > 2) {
                    fThread = 0;
                    break;
                }

                movingAngle = atan2(wayPoint_y[iWaypoint] - ctr.robot.y, wayPoint_x[iWaypoint] - ctr.robot.x) * 180.0 / M_PI;
                //printf("waypoint (%d, %d), robot (%d, %d), angle %.3f\n", wayPoint_x[iWaypoint], wayPoint_y[iWaypoint],ctr.robot.x, ctr.robot.y, movingAngle);
                change_moving_angle (  movingAngle );

                dis2 = sqrt ( pow ( wayPoint_x[iWaypoint] - ctr.robot.x, 2 ) + pow ( wayPoint_y[iWaypoint] - ctr.robot.y, 2 ) );
                if (dis2 < 30)
                    iWaypoint ++;
                else {                        // if robot has not moved cargo

                }
                break;
        }

        //printf("time: %.3f\n", timeElapsed);
        /* calc. field in x-y directions */
        if (fThread) {
            if (fKey) {
                change_moving_angle (  directionCode * 90.0 );
                fKey = 0;                   // reset key flag
            }
            my_sleep(10);
        }

    }
    /* stop low-level coil thread: output sawtooth wave */
    stop_coil_thread ();                   // coil.cpp
    printf("at the end of actuation_THREAD.\n");
}

/* start or stop the actuation thread */
void on_tB_actuation_toggled (GtkToggleButton *togglebutton, gpointer data) {
    int d = gtk_toggle_button_get_active (togglebutton);

    if (d && fThread) {
        printf("Error: an actuation thread is already running.\n");
        my_sleep (1000);
    } else {
        fThread = d;
        if (d) {
            pthread_t actuationThread;
            pthread_create( &actuationThread, NULL, actuation_THREAD, NULL);
        }
    }
}

/* thread of actuation for rectangular robot */
static void* actuationRectRobot_THREAD ( void *threadid ) {
    printf("at the start of actuationRectRobot_THREAD.\n");

    /* variable definition */
    float movingAngle = 0.0;            // desired moving angle of robot in XY plane degrees
    double presentTime = 0;

    MMC_Controller ctr;                 // get an instance of the controller
    ctr.get_latest_pos (0);             // init. run to get robot and cargo pos.

    float dis2 = 0.0;                   // distance from robot to destination
    float contactPos[2] = {0,0};        // position when contact happens
    double contactTime = 0.0;            // time when contact happens; has to be "double", otherwise will have error when computing with presentTime
    int nSwitch = 1;                    // no. of switch to push cargo from stuck; start from 1, so that robot moves a distance before oscillation

    int wayPoint_x[3] = {380, 160, 130};
    int wayPoint_y[3] = {150, 150, 480-177};
    int iWaypoint = 0;

    int iCargo = get_cargo_type ();         // get cargo type specified on GUI, vision.cpp

    /* start low-level coil thread: output sawtooth wave */
    start_coil_thread ();                   // coil.cpp
    pause_coil_output (1);
    my_sleep(1000);
    pause_coil_output (0);
    switch_to_gradient_mode ();         // coil.cpp

    while (fThread) {
        presentTime = get_present_time ();

        int rst = ctr.get_latest_pos (1);       // get current position info. of robot and cargo

        switch (ctr.state) {
            /* Step 0: move robot to above cargo (along y axis) */
            case 0:
                if (rst) {                      // if cargo detection is valid ...
                    ctr.update_goal_info_using_cargo_pos_for_rect_robot();
                    change_moving_angle ( ctr.angle );

                    if (ctr.dis < 20) {                     // if reaching the goal
                        ctr.state = 1;
                        pause_coil_output (1);              // stop walking motion
                        printf("reach state 1.\n");
                        my_sleep (1000);
                        pause_coil_output (0);              // resume walking motion
                    }
                } else {
                    // in this state, just igore abnormal detection
                }
                break;
            /* Step: align robot with cargo's orientation */
            case 1:
                pause_coil_output (1);                  // stop walking motion (coil.cpp)
                ctr.set_cargo_as_goal ();
                change_moving_angle ( ctr.angle );
                printf("stage 1: change move anlge %.3f\n", ctr.angle);
                my_sleep (1000);                        // wait for robot to rotate

                ctr.state = 2;
                printf("reach state 2.\n");
                pause_coil_output (0);                  // resume walking motion
                break;
            /* Step: contact cargo */
            case 2:
                if ( ! ctr.fContact ) {       // if contact has not happended ...
                    printf("robot has NOT touched cargo.\n");
                    if ( rst ) {                            // if cargo detection is valid ...
                        printf("cargo detection is valid.\n");
                        ctr.set_cargo_as_goal ();
                        change_moving_angle ( ctr.angle );
                    } else {                        // if cargo detection is not valid ...
                        printf("cargo detection is invalid.\n");
                        rst = ctr.check_contact (iCargo);     // check if contact happened
                        if (rst) {                      // if robot has contacted cargo ...
                            contactPos[0] = ctr.robot.x;  // record the robot position when contact happens
                            contactPos[1] = ctr.robot.y;
                            contactTime = presentTime;
                            pause_coil_output (1);
                            printf("robot has touched cargo at time %.3f.\n", contactTime);
                            my_sleep(1000);
                            pause_coil_output (0);
                        }                               // otherwise just igore this detection
                    }
                } else {                      // if contact has happened
                    float tempDis = sqrt  (   pow ( ctr.robot.x - contactPos[0], 2 )
                                            + pow ( ctr.robot.y - contactPos[1], 2 ) ); // dis. between current pos. and contact pos.
                    printf("dis. between current pos. and contact pos. %.3f.\n", tempDis);
                    if ( (tempDis > 30) && (ctr.robot_away_from_init_pos()) ) {
                        pause_coil_output (1);
                        ctr.state = 3;
                        printf("reach state 3.\n");
                        my_sleep (1000);
                        pause_coil_output (0);
                        //switch_to_uniform_mode ();         // coil.cpp
                    } else {
                        if ( ( (int)(presentTime - contactTime) ) > nSwitch * 2 ) {
                            nSwitch ++;
                            printf("nSwitch: %d, present time: %.3f, contact time: %.3f.\n", nSwitch, presentTime, contactTime);
                            if ( nSwitch / 2 % 2 == 0 )
                                change_moving_angle ( ctr.angle + 15 );
                            else
                                change_moving_angle ( ctr.angle - 15 );     // change pushing angle to overcome friction
                        }
                    }
                }
                break;
            /* Step: pass waypoints */
            case 3:
                if (iWaypoint > 2) {
                    fThread = 0;
                    break;
                }

                movingAngle = atan2(wayPoint_y[iWaypoint] - ctr.robot.y, wayPoint_x[iWaypoint] - ctr.robot.x) * 180.0 / M_PI;
                printf("waypoint (%d, %d), robot (%d, %d), angle %.3f\n", wayPoint_x[iWaypoint], wayPoint_y[iWaypoint],ctr.robot.x, ctr.robot.y, movingAngle);
                change_moving_angle (  movingAngle );

                dis2 = sqrt ( pow ( wayPoint_x[iWaypoint] - ctr.robot.x, 2 ) + pow ( wayPoint_y[iWaypoint] - ctr.robot.y, 2 ) );
                if (dis2 < 30)
                    iWaypoint ++;
                else {                        // if robot has not moved cargo

                }
                break;
        }
        if (fThread) {
            my_sleep(10);
        }
    }
    /* stop low-level coil thread: output sawtooth wave */
    stop_coil_thread ();                   // coil.cpp
    printf("at the end of actuationRectRobot_THREAD.\n");
}

/* start or stop the actuation thread for rectangular robot*/
void on_tB_rectangularRobot_toggled (GtkToggleButton *togglebutton, gpointer data) {
    int d = gtk_toggle_button_get_active (togglebutton);

    if (d && fThread) {
        printf("Error: an actuation thread is already running.\n");
        my_sleep (1000);
    } else {
        fThread = d;
        if (d) {
            pthread_t actuationRectRobotThread;
            pthread_create( &actuationRectRobotThread, NULL, actuationRectRobot_THREAD, NULL);  //start vision thread
        }
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
    fKey = 1;                   // a key is pressed
}
