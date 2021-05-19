#ifndef DISPLAYDRIVER_H
#define DISPLAYDRIVER_H


#include <Nextion.h> 
#include "ethernet.h"

extern volatile uint16_t network_port;
extern volatile uint32_t temp_sealing;
extern volatile uint32_t temp_preheat;
extern volatile uint32_t temp_measured;
extern volatile float current_rms;
extern volatile float voltage_rms;
extern volatile uint8_t static_ip_arr[IP_ARRAY_SIZE];

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

/********************************************************************
   Function:    InitDisplay()
   Precondition: 
   Input:      
   Returns:

   Side Effects: Uses Serial1 with 115000 baudrate

   Overview:    Initializes display module

   Note:     

 ********************************************************************/
void InitDisplay();


/********************************************************************
   Function:    updateDisplay()
   Precondition: Display must be initialized
   Input:      current state machine state;
   			   current input states
   Returns:

   Side Effects:

   Overview:   Updates current display page

   Note:     

 ********************************************************************/
void updateDisplay(int state, int input_start, int input_preheat, int input_sealing);


/********************************************************************
   Function:    eventCheck()
   Precondition: Display must be initialized
   Input:      
   Returns:

   Side Effects:

   Overview:    Checks interrupt flags from display events

   Note:     

 ********************************************************************/
void eventCheck();


/********************************************************************
   Function:    errorPage()
   Precondition: Display must be initialized
   Input:       error code
   Returns:

   Side Effects:

   Overview:    Forces error page with error message according
   				to error code

   Note:     

 ********************************************************************/
void errorPage(int8_t error_code);


/********************************************************************
   Function:    resetDisplay()
   Precondition: Display must be initialized
   Input:      
   Returns:

   Side Effects:

   Overview:    Resets display parameters and forces reboot

   Note:     

 ********************************************************************/
void resetDisplay();

#endif
