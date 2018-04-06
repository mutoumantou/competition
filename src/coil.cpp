#include "coil.hpp"

static int fThread = 0;                 // thread flag
static int fCoilBusy = 0;              // whether or not a new signal is being delivered to the coil
static int moveDir = 0, fMoveDir = 0;                 // 0: move forward; 1: move backward
static float ampXY = 1.0;             //field amplitude in XY plane in voltage
static float tiltAngle = 45.0;      // tilting angle
static float ampZ  = ampXY * tand(tiltAngle);

static float freq = 10.0;               // actuation frequency
static float periodTime = 1.0 / freq;   // time taken per period

/* class definition */
/* CLASS: direct control of physical coil system */
class Coil_System {
    public:
        Coil_System ( void );
        void set_uniform_field_volt ( float data[3] );
        void set_z_field_volt (float data);
        void output_signal ( void );
        void set_angle (float data);
        void rotate_to_new_angle ( void );
        void stop_output (void);
        void add_gradient_output (void);        // add gradient along the moving direction
    private:
        float uniformV[3];
        float angle, angleOld;            // field angle in X-Y plane in degrees
        int fGradient;                  // whether or not add gradient to output
};

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
        uniformV[0] = 1 * ampXY * i * 0.1;
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
    fGradient = 1;
    for (int i = 0; i < abs(del); i ++) {
        angleOld = angleOld + step;
        uniformV[0] = cosd(angleOld) * ampXY;
        uniformV[1] = sind(angleOld) * ampXY;
        uniformV[2] = 0.0;
        output_signal ();
        my_sleep(10);
    }
    fGradient = 0;
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

/* change actuation frequency in Hz */
void on_spin_freq_changed (GtkEditable *editable, gpointer user_data) {
    freq = gtk_spin_button_get_value ( GTK_SPIN_BUTTON(editable) );
    periodTime = 1.0 / freq;            // update period time
}

/* declare an instance of coil system class */
static Coil_System coil;

/* THREAD: output sawtooth wave */
static void* coil_THREAD ( void *threadid ) {
    printf("at the start of coil_THREAD.\n");

    /* init. s826 board */
    int initFlag = s826_init();
    printf("s826 board init. result: %d (should be 0)\n", initFlag); // result should be 0

    /* time related variables */
    double startTime = 0, presentTime = 0, timeElapsed = 0;
    startTime = get_present_time ();

    while (fThread) {
        /* update time elapsed within a period */
        presentTime = get_present_time ();
        timeElapsed = presentTime - startTime;      // calc. how much time has passed
        while ( timeElapsed >= periodTime ) {
          timeElapsed = timeElapsed - periodTime;
          startTime   = presentTime - timeElapsed;
        }

        /* decide z field amplitude based on time, output physical signals */
        while (fCoilBusy);
        fCoilBusy = 1;
        while (fMoveDir);
        fMoveDir = 1;
        coil.set_z_field_volt ( (moveDir * 2 - 1 ) * ampZ * timeElapsed / periodTime );
        fMoveDir = 0;
        coil.output_signal ();
        fCoilBusy = 0;
        my_sleep(10);
    }
    coil.stop_output();
    s826_close();
    printf("at the start of coil_THREAD.\n");
}

/* start low-level coil thread: output sawtooth wave */
void start_coil_thread (void) {
    if (fThread) {
        printf("Error: the low-level coil thread is already running, check your code.\n");
        my_sleep(1000);
    } else {
        fThread = 1;
        pthread_t coilThread;
        pthread_create( &coilThread, NULL, coil_THREAD, NULL);  //start vision thread
    }
}

/* stop low-level coil thread: output sawtooth wave */
void stop_coil_thread (void) {
    fThread = 0;
}

/* change moving angle */
void change_moving_angle ( float data ) {
    while (fCoilBusy);
    fCoilBusy = 1;

    /* stop moving */
    coil.set_z_field_volt ( 0 );
    coil.output_signal ();

    /* move to new angle */
    coil.set_angle ( data );       // set moving angle to coil
    coil.rotate_to_new_angle ();
    fCoilBusy = 0;
}

/* changing the robot between moving forward or backward */
void change_moving_dir (int data) {
    while (fMoveDir);
    fMoveDir = 1;
    moveDir = data;
    fMoveDir = 0;
}
