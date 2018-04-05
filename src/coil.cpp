#include "coil.hpp"

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
