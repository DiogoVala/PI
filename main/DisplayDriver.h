#ifndef DISPLAYDRIVER_H
#define DISPLAYDRIVER_H


#include <Nextion.h> 

extern volatile uint32_t temp_sealing;
extern volatile uint32_t temp_preheat;
extern volatile uint32_t temp_measured;
extern volatile float current_rms;
extern volatile float voltage_rms;

void InitDisplay();
void updateHome(int state, int input_start,int input_preheat, int input_sealing);
void updateGraphs();
void eventCheck();
void errorPage();
void resetDisplay();

#endif
