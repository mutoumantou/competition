#ifndef ACTUATION
#define ACTUATION

#include "general_header.hpp"
#include "s826.hpp"

void on_tB_actuation_toggled (GtkToggleButton *togglebutton, gpointer data);
void set_directionCode (int keycode);

#endif
