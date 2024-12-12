#ifndef PROTOS_H_
#define PROTOS_H_

#include "defines.h"

void RAM (SpinnerRTTYTest)(void);
void core1_entry();



/* conswrapper.c */

void PushErrorMessage(int id);
void PushStatusMessage(void);

#endif
