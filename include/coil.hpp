#ifndef COIL
#define COIL

#include "general_header.hpp"
#include "s826.hpp"
#include "vision.hpp"

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

/* send signal to amplifiers */
#define Coil_PX 0
#define Coil_NX 3
#define Coil_PY 4
#define Coil_NY 1
#define Coil_PZ 2
#define Coil_NZ 5

#endif
