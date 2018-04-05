#ifndef ACTUATION
#define ACTUATION

#include "general_header.hpp"
#include "s826.hpp"
#include "vision.hpp"
#include "controller.hpp"
#include "coil.hpp"

extern "C" {
    void on_tB_actuation_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_spin_freq_changed (GtkEditable *editable, gpointer user_data);
    void on_spin_amp_changed  (GtkEditable *editable, gpointer user_data);
    void on_spin_tiltingAngle_changed (GtkEditable *editable, gpointer user_data);
    void on_toggle_gradient_toggled (GtkToggleButton *togglebutton, gpointer data);
}

/* function declaration */
void set_directionCode (int keycode);

#endif
