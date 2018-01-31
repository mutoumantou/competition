#ifndef GUIMASTER
#define GUIMASTER

#include "general_header.hpp"

void GUI_master_activate (GtkImage *vidWindow, GtkLabel *timeLabel);        // start updating GUI
void GUI_master_deactivate(void);       // stop updating GUI

#endif
