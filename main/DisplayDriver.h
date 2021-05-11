#ifndef DISPLAYDRIVER_H
#define DISPLAYDRIVER_H


#include <Nextion.h> 

extern volatile uint16_t network_port;
extern volatile uint32_t temp_sealing;
extern volatile uint32_t temp_preheat;
extern volatile uint32_t temp_measured;
extern volatile float current_rms;
extern volatile float voltage_rms;

enum Page {
	pg_START,
	pg_HOME,
	pg_PARAM,
	pg_GRAPH1,
	pg_GRAPH2,
	pg_GRAPH3,
	pg_INFO,
	pg_HELP,
	pg_ERROR,
	pg_NETWORK
};

void InitDisplay();
//void sendCommand(const char* text);
void updateDisplay(int state, int input_start, int input_preheat, int input_sealing);
void eventCheck();
void errorPage(int8_t error_code);
void resetDisplay();

#endif
