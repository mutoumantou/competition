#include "keyboard.hpp"

gboolean key_event (GtkWidget *widget, GdkEventKey *event) {
    //printf ("key pressed %s\n", gdk_keyval_name( event->keyval ) );
    int keycode = -1;

    switch (event->keyval) {
        case 65363: keycode = 0; break;
        case 65362: keycode = 1; break;
        case 65361: keycode = 2; break;
        case 65364: keycode = 3; break;
        case    32: keycode = -1; break;
    }

    set_directionCode (keycode);
    return TRUE;
}

gboolean key_event_release (GtkWidget *widget, GdkEventKey *event) {
    return TRUE;
}
