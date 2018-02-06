#include "actuation.hpp"

static int fThread = 0;             // flag of thread running
static int directionCode = -1;      // -1: neutral; 0: +x; 1: +y; 2: -x; 3: -y
static int fKey = 0;                // if a key has been pressed, request reviewing direction

/* send signal to amplifiers */
#define Coil_PX 0
#define Coil_NX 3
#define Coil_PY 4
#define Coil_NY 1
#define Coil_PZ 2
#define Coil_NZ 5


static void send_signal_to_amplifier (float outputV[3]) {
    s826_aoPin(Coil_PX, 2, outputV[0]);
    s826_aoPin(Coil_NX, 2, outputV[0]);
    s826_aoPin(Coil_PY, 2, outputV[1]);
    s826_aoPin(Coil_NY, 2, outputV[1]);
    s826_aoPin(Coil_PZ, 2, outputV[2]);
    s826_aoPin(Coil_NZ, 2, outputV[2]);
    //printf("output %.3f %.3f %.3f.\n", outputV[0], outputV[1], outputV[2]);
}

/* dampe rotation, otherwise robot will fly */
static void damp_rotation (float oldOutput[3], float newOutput[3]) {
    float del[3] = {0,0,0};
    for (int i = 0; i < 3; i ++)
        del[i] = 0.01 * (newOutput[i] - oldOutput[i]);

    for (int i = 0; i < 100; i ++) {
        for (int j = 0; j < 3; j ++)
            oldOutput[j] = oldOutput[j] + del[j];
        send_signal_to_amplifier (oldOutput);
        my_sleep(10);
    }
    printf("at the end of damp\n");
}

/* thread of actuation */
static void* actuation_THREAD ( void *threadid ) {
    printf("at the start of actuation_THREAD.\n");
    int initFlag = s826_init();
    printf("init result %d\n", initFlag);

    /* variable definition */
    float freq = 10;                // frequency of vibration
    float periodTime = 1.0 / freq;  // time per period
    float tiltAngle = 20;           // tilting angle of degrees
    float ampXY = 2;                // field amplitude in voltage
    float ampZ  = ampXY * tand(tiltAngle);
    float outputV[3] = {0,0,0};     // output voltage for x, y, z
    float angle = 0;                // moving angle in x-y plane

    double startTime = 0, presentTime = 0, timeElapsed = 0;
    startTime = get_present_time ();
    while (fThread) {
        presentTime = get_present_time ();
        timeElapsed = presentTime - startTime;
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
                outputV[0] = 0;
                outputV[1] = 0;
                outputV[2] = 0;

            } else {
                outputV[2] = 0;
                angle = directionCode * 90.0;
                float newOutput[3] = {0,0,0};
                newOutput[0] = ampXY * cosd(angle);
                newOutput[1] = ampXY * sind(angle);
                damp_rotation (outputV, newOutput);
                printf("outputv %.3f %.3f %.3f\n", outputV[0], outputV[1], outputV[2]);
                //for (int i = 0; i < 3; i ++)
                //    outputV[i] = newOutput[i];
            }
            fKey = 0;                   // reset key flag
        }

        /* decide z field amplitude based on time */
        if (directionCode != -1)
            outputV[2] = -1.0 * ampZ * timeElapsed / periodTime;         // make the object tail tilt up

        send_signal_to_amplifier (outputV);
        my_sleep(10);
    }

    outputV[0] = 0;outputV[1] = 0;outputV[2] = 0;
    send_signal_to_amplifier (outputV);
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
