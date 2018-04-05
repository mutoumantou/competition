#include "actuation.hpp"

static int fThread = 0;             // flag of thread running
static int fGradient = 0;           // 0: use uniform field; 1: use gradient field
static int directionCode = 0;      // -1: neutral; 0: +x; 1: +y; 2: -x; 3: -y
static int fKey = 0;                // if a key has been pressed, request reviewing direction
static float freq = 10.0;               // actuation frequency
static float periodTime = 1.0 / freq;   // time taken per period
static float ampXY = 1.0;             //field amplitude in XY plane in voltage
static float tiltAngle = 45.0;      // tilting angle
static float ampZ  = ampXY * tand(tiltAngle);

/* thread of actuation */
static void* actuation_THREAD ( void *threadid ) {
    printf("at the start of actuation_THREAD.\n");
    int initFlag = s826_init();           // init. s826 board
    printf("s826 board init. result: %d\n", initFlag); // result should be 0

    /* variable definition */
    Coil_System coil;                     // get an instance of coil system (class)
    float movingAngle = 0.0;              // desired moving angle of robot in XY plane degrees
    float zOutput = 0.0;                  // output voltage for z coil
    double startTime = 0, presentTime = 0, timeElapsed = 0;
    startTime = get_present_time ();

    MMC_Controller ctr;                 // get an instance of the controller
    ctr.get_latest_pos (0);             // init. run to get robot and cargo pos.

    float dis2 = 0.0;                   // distance from robot to destination
    float contactPos[2] = {0,0};        // position when contact happens
    double contactTime = 0.0;            // time when contact happens; has to be "double", otherwise will have error when computing with presentTime
    int nSwitch = 0;                    // no. of switch in order to push cargo from stuck

    int wayPoint_x[3] = {500, 160, 163};
    int wayPoint_y[3] = {150, 150, 480-177};
    int iWaypoint = 0;
    int fCargoMove = 0;                 // whether or not robot has successfully moved cargo

    int iCargo = get_cargo_type ();         // get cargo type specified on GUI, vision.cpp

    while (fThread) {
        presentTime = get_present_time ();          // get present time
        //printf("present time: %.3f.\n", presentTime);
        timeElapsed = presentTime - startTime;      // calc. how much time has passed
        if (timeElapsed >= periodTime) {
            while (timeElapsed >= periodTime) {
              timeElapsed = timeElapsed - periodTime;
            }
            startTime = presentTime - timeElapsed;
        }

        int rst = ctr.get_latest_pos (1);           // get current position info. of robot and cargo
        //printf("robot (%d, %d), cargo (%d, %d)\n", robotPos.x, robotPos.y, cargoPos.x, cargoPos.y);

        switch (ctr.state) {
            /* Step: move robot to cargo */
            case 0:
                if (rst) {               // if cargo detection is valid ...
                    if (iCargo != 0) {        // if cargo not circle
                        ctr.update_goal_info_using_cargo_pos();
                        coil.set_angle ( ctr.angle );       // set moving angle to coil
                        coil.rotate_to_new_angle ();

                        if (ctr.dis < 40) {                   // if reaching the goal
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
                ctr.set_cargo_as_goal ();
                coil.set_angle ( ctr.angle );       // set moving angle to coil
                coil.rotate_to_new_angle ();

                /* check the alignment between robot and cargo */
                rst = ctr.get_latest_pos (1);           // get robot and cargo position after rotation
                my_sleep (1000);                        // wait for user to review terminal output

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
                        coil.set_angle ( ctr.angle );       // set moving angle to coil
                        coil.rotate_to_new_angle ();
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
                            if ( nSwitch / 2 % 2 == 0 ) {
                                coil.set_angle ( ctr.angle + 30 );       // set moving angle to coil

                            } else {
                                coil.set_angle ( ctr.angle - 30 );       // set moving angle to coil
                            }
                            coil.rotate_to_new_angle ();
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
                coil.set_angle ( movingAngle );
                coil.rotate_to_new_angle ();
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
                if (directionCode == -1) {
                  coil.stop_output ();
                } else {
                  coil.set_angle (directionCode * 90.0);
                  coil.rotate_to_new_angle ();
                }
                fKey = 0;                   // reset key flag
            }

            /* decide z field amplitude based on time */
            if (directionCode != -1) {
              zOutput = -1.0 * ampZ * timeElapsed / periodTime;         // make the object tail tilt up
              coil.set_z_field_volt ( zOutput );
            }
            coil.output_signal ();
            my_sleep(10);
        }

    }
    coil.stop_output();
    s826_close();
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
    fKey = 1;                   // a key is pressed
}

/* change actuation frequency in Hz */
void on_spin_freq_changed (GtkEditable *editable, gpointer user_data) {
    freq = gtk_spin_button_get_value ( GTK_SPIN_BUTTON(editable) );
    periodTime = 1.0 / freq;            // update period time
}

/* change actuation amplitude in mT */
void on_spin_amp_changed  (GtkEditable *editable, gpointer user_data) {
    float d = gtk_spin_button_get_value ( GTK_SPIN_BUTTON(editable) );
    ampXY = d / 5.0;              // convert mT to voltage
    ampZ  = ampXY * tand(tiltAngle);
}

/* change tilting angle in degrees */
void on_spin_tiltingAngle_changed (GtkEditable *editable, gpointer user_data) {
    tiltAngle = gtk_spin_button_get_value ( GTK_SPIN_BUTTON(editable) );
    ampZ  = ampXY * tand(tiltAngle);
}

void on_toggle_gradient_toggled (GtkToggleButton *togglebutton, gpointer data) {
    fGradient = gtk_toggle_button_get_active (togglebutton);
}
