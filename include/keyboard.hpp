#ifndef KEYBOARDCONTROL
#define KEYBOARDCONTROL

#include "general_header.hpp"
#include <gdk/gdkkeysyms.h>
#include "actuation.hpp"

gboolean key_event (GtkWidget *widget, GdkEventKey *event);
gboolean key_event_release (GtkWidget *widget, GdkEventKey *event);

#endif
