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

/* send signal to amplifiers */
#define Coil_PX 0
#define Coil_NX 3
#define Coil_PY 4
#define Coil_NY 1
#define Coil_PZ 2
#define Coil_NZ 5

/* class definition */
/* constructor */
Coil_System :: Coil_System ( void ) {
    for (int i = 0; i < 3; i ++) {
        uniformV[i] = 0;
    }
    angle = 0;
    angleOld = 0;
    fGradient = 0;
    /* ensure robot is init. aligned with +x */
    for (int i = 0; i < 10; i ++) {
        uniformV[0] = ampXY * i * 0.1;
        output_signal ();
        my_sleep(50);
    }
    //uniformV[0] = 0;
    //output_signal ();
}

void Coil_System :: set_uniform_field_volt ( float data[3] ) {
    for (int i = 0; i < 3; i ++) {
        uniformV[i] = data[i];
    }
}

/* set the z field strength to tilt robot */
void Coil_System :: set_z_field_volt (float data) {
    uniformV[2] = data;
}

void Coil_System :: output_signal ( void ) {
    if (!fGradient) {
        s826_aoPin(Coil_PZ, 2, uniformV[2]);     // output z first so that when clearing field the robot will not stand up
        s826_aoPin(Coil_NZ, 2, uniformV[2]);

        s826_aoPin(Coil_PX, 2, uniformV[0]);
        s826_aoPin(Coil_NX, 2, uniformV[0]);
        s826_aoPin(Coil_PY, 2, uniformV[1]);
        s826_aoPin(Coil_NY, 2, uniformV[1]);

    } else {
        if (uniformV[2] >= 0) {
            s826_aoPin(Coil_PZ, 2, uniformV[2]);
            s826_aoPin(Coil_NZ, 2, 0);
        } else {
            s826_aoPin(Coil_PZ, 2, 0);
            s826_aoPin(Coil_NZ, 2, uniformV[2]);
        }

        if (uniformV[0] >= 0) {
            s826_aoPin(Coil_PX, 2, uniformV[0]);
            s826_aoPin(Coil_NX, 2, 0);
        } else {
            s826_aoPin(Coil_PX, 2, 0);
            s826_aoPin(Coil_NX, 2, uniformV[0]);
        }

        if (uniformV[1] >= 0) {
            s826_aoPin(Coil_PY, 2, uniformV[1]);
            s826_aoPin(Coil_NY, 2, 0);
        } else {
            s826_aoPin(Coil_PY, 2, 0);
            s826_aoPin(Coil_NY, 2, uniformV[1]);
        }
    }

    //printf("output %.3f %.3f %.3f %.3f %.3f %.3f.\n", outputV[0], outputV[1], outputV[2],outputV[3], outputV[4], outputV[5]);
    fGradient = 0;
}

void Coil_System :: set_angle (float data) {
    angleOld = angle;           // remember the last angle
    angle = data;
}

void Coil_System :: rotate_to_new_angle ( void ) {
    // angle is in range of [0, 360)
    float del = angle - angleOld;
    if (del > 180)
        del = del - 360;
    if (del < -180)
        del = del + 360;

    int step = 1;
    if (del < 0)
        step = -1;

    /* gently rotate robot */
    uniformV[2] = 0;

    for (int i = 0; i < abs(del); i ++) {
        angleOld = angleOld + step;
        uniformV[0] = cosd(angleOld) * ampXY;
        uniformV[1] = sind(angleOld) * ampXY;
        uniformV[2] = 0.0;
        output_signal ();
        my_sleep(10);
    }
    angleOld = angle;
}

void Coil_System :: stop_output (void) {
    for (int i = 0; i < 3; i ++) {
        uniformV[i] = 0;
    }
    angle = 0;
    angleOld = 0;
    output_signal ();
}

void Coil_System :: add_gradient_output (void) {
    fGradient = 1;
}

/* define a class for the controller */
class MMC_Controller {
  public:
    MMC_Controller (void);      // constructor
    int get_latest_pos (int data);  // get latest position info from vision.cpp
    int check_contact (void);         // check if robot has touched cargo
    float dis, angle;                 // distance from robot to cargo, desired moving angle
    cv :: Point robot, cargo;         // center point position of robot and cargo
    int fContact;
  private:
    cv :: Point preRobot, preCargo;   // previous center point information of robot and cargo

};

MMC_Controller :: MMC_Controller () {
  robot.x = 0;
  robot.y = 0;
  cargo.x = 0;
  cargo.y = 0;
  preRobot.x = 0;
  preRobot.y = 0;
  preCargo.x = 0;
  preCargo.y = 0;
  dis = 0.0; angle = 0.0;
  fContact = 0;
}

/*
output: 1: valid new position; 0: cargo pos not valid
intput: 0: 1st run, use for preCargo and preRobot; 1: following runs
robot pos. is considered to be always valid
*/
int MMC_Controller :: get_latest_pos (int data) {
  if (data) {
    return_center_pt_info ( &robot, &cargo );
    preRobot.x = robot.x;
    preRobot.y = robot.y;
    if ( abs(cargo.x - preCargo.x) < 20 && abs(cargo.y - preCargo.y) < 20 ) {
        preCargo.x = cargo.x;
        preCargo.y = cargo.y;
        dis   = sqrt  ( pow ( cargo.x - robot.x, 2 ) + pow ( cargo.y - robot.y, 2 ) );
        angle = atan2 ( cargo.y - robot.y, cargo.x - robot.x) * 180.0 / M_PI;
        return 1;
    } else {        // if cargo pos. is not valid, do not update dis.
        cargo.x = preCargo.x;
        cargo.y = preCargo.y;
        return 0;
    }
  }
  return_center_pt_info ( &preRobot, &preCargo );
  return 1;
}

/*
output: 1: contact happens; 0: contact not happen
this fun. is only called when cargo is not correctly detected
*/
int MMC_Controller :: check_contact (void) {
  if (dis < 30)           // if pre. distance is < threshold, then contact happened
    fContact = 1;
                          // else, just a wrong detection, ignore this detection
  return fContact;
}

/* thread of actuation */
static void* actuation_THREAD ( void *threadid ) {
    printf("at the start of actuation_THREAD.\n");
    int initFlag = s826_init();           // init. s826 board
    printf("init result %d\n", initFlag); // result should be 0

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

    int fCargoMove = 0;                 // whether or not robot has successfully moved cargo
    while (fThread) {
      presentTime = get_present_time ();
      timeElapsed = presentTime - startTime;

      int rst = ctr.get_latest_pos (1); // get current position info. of robot and cargo
      //printf("robot (%d, %d), cargo (%d, %d)\n", robotPos.x, robotPos.y, cargoPos.x, cargoPos.y);

      if ( ! ctr.fContact ) {         // if contact has not happended ...
        if ( ! rst ) {                  // if cargo detection is not valid ...
          rst = ctr.check_contact ();     // check if contact happened
          if (rst) {                      // if robot has contacted cargo ...
            contactPos[0] = ctr.robot.x;  // record the robot position when contact happens
            contactPos[1] = ctr.robot.y;
            printf("robot has touched cargo.\n");
          }                               // otherwise just igore this detection
        } else {                        // if cargo detection is valid ...
          coil.set_angle ( ctr.angle );   // set moving angle to coil
          coil.rotate_to_new_angle ();
        }
      } else {                      // if contact has happened
        if ( fCargoMove ) {             // if robot has moved cargo
          movingAngle = atan2(240 - ctr.robot.y, 250 - ctr.robot.x) * 180.0 / M_PI;
          coil.set_angle ( movingAngle );
          coil.rotate_to_new_angle ();
          dis2 = sqrt ( pow ( 250 - ctr.robot.x, 2 ) + pow ( 240 - ctr.robot.y, 2 ) );
          if (dis2 < 15)
              fThread = 0;
        } else {                        // if robot has not moved cargo
          float tempDis = sqrt  ( pow ( ctr.robot.x - contactPos[0], 2 ) + pow ( ctr.robot.y - contactPos[1], 2 ) );
          if (tempDis > 40)
            fCargoMove = 1;
        }
      }
      if (timeElapsed >= periodTime) {
          while (timeElapsed >= periodTime) {
              timeElapsed = timeElapsed - periodTime;
          }
          startTime = presentTime - timeElapsed;
      }
      //printf("time: %.3f\n", timeElapsed);
      /* calc. field in x-y directions */
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
}

void on_toggle_gradient_toggled (GtkToggleButton *togglebutton, gpointer data) {
    fGradient = gtk_toggle_button_get_active (togglebutton);
}
