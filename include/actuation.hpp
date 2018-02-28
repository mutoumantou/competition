#ifndef ACTUATION
#define ACTUATION

#include "general_header.hpp"
#include "s826.hpp"
#include "vision.hpp"


extern "C" {
    void on_tB_actuation_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_spin_freq_changed (GtkEditable *editable, gpointer user_data);
    void on_spin_amp_changed  (GtkEditable *editable, gpointer user_data);
    void on_spin_tiltingAngle_changed (GtkEditable *editable, gpointer user_data);
    void on_toggle_gradient_toggled (GtkToggleButton *togglebutton, gpointer data);
}

/* function declaration */

void set_directionCode (int keycode);

/* class definitoin */
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

#endif
