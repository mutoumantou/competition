#ifndef ACTUATION
#define ACTUATION

#include "general_header.hpp"
#include "s826.hpp"
#include "vision.hpp"
#include "controller.hpp"
#include "coil.hpp"

extern "C" {
    void on_tB_actuation_toggled        (GtkToggleButton *togglebutton, gpointer data);
    void on_tB_rectangularRobot_toggled (GtkToggleButton *togglebutton, gpointer data);
}

/* function declaration */
void set_directionCode (int keycode);

#endif
