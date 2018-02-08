#ifndef ACTUATION
#define ACTUATION

#include "general_header.hpp"
#include "s826.hpp"

/* function declaration */
void on_tB_actuation_toggled (GtkToggleButton *togglebutton, gpointer data);
void set_directionCode (int keycode);

/* class definitoin */
class Coil_System {
    public:
        Coil_System ( void );
        void set_uniform_field_volt ( float data[3] );
        void set_z_field_volt (float data);
        void set_gradient_field_volt ( float data[6] );
        void output_signal ( void );
        void set_angle (float data);
        void rotate_to_new_angle ( void );
        void stop_output (void);
        void add_gradient_output (void);        // add gradient along the moving direction
    private:
        float outputV[6];       // output voltage for +x, -x, +y, -y, +z, -z
        float uniformV[3];
        float gradientV[6];
        float angle, angleOld;            // field angle in X-Y plane in degrees
        int fGradient;                  // whether or not add gradient to output
};

#endif
