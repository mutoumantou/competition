#ifndef GUIMASTER
#define GUIMASTER

#include <pthread.h>                    // for using thread
#include <stdio.h>                      // for printf
#include <time.h>

void GUI_master_activate (void);        // start updating GUI
void GUI_master_deactivate(void);       // stop updating GUI

#endif
