#include "keyboard.hpp"

gboolean key_event (GtkWidget *widget, GdkEventKey *event) {
    printf ("key pressed %s\n", gdk_keyval_name (event->keyval));
    return TRUE;
}
