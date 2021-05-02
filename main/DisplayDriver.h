#ifndef DISPLAY_H
#define DISPLAY_H

#include <Nextion.h>

extern volatile uint32_t temp_user_setpoint;
extern volatile uint32_t temp_preheat;
extern volatile uint32_t temp_measured;

void InitDisplay();
void checkDisplayEvent();
void sendGraphVal();

#endif