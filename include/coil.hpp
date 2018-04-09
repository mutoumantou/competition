#ifndef COIL
#define COIL

#include "general_header.hpp"
#include "s826.hpp"
#include "vision.hpp"

/* send signal to amplifiers */
#define Coil_PX 0
#define Coil_NX 3
#define Coil_PY 4
#define Coil_NY 1
#define Coil_PZ 2
#define Coil_NZ 5

extern "C" {
    /* change actuation amplitude in mT */
    void on_spin_amp_changed  (GtkEditable *editable, gpointer user_data);
    /* change tilting angle in degrees */
    void on_spin_tiltingAngle_changed (GtkEditable *editable, gpointer user_data);
    /* change sawtooth frequency */
    void on_spin_freq_changed (GtkEditable *editable, gpointer user_data);
    /* use gradient or uniform field */
    void on_toggle_gradient_toggled (GtkToggleButton *togglebutton, gpointer data);
}

/* start low-level coil thread: output sawtooth wave */
void start_coil_thread (void);
/* stop low-level coil thread: output sawtooth wave */
void stop_coil_thread (void);
/* change moving angle */
void change_moving_angle ( float data );
/* changing the robot between moving forward or backward */
void change_moving_dir (int data);

void pause_coil_output (int data);

void switch_to_gradient_mode (void);
void switch_to_uniform_mode (void);

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

};


#endif
