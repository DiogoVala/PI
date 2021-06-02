/*
   File:   display.h
   Author: Diogo Vala

   Overview: Display control and interaction
*/

#ifndef DISPLAYDRIVER_H
#define DISPLAYDRIVER_H

#include <Nextion.h> 
#include "ethernet.h"
#include "errors.h"

#define DISPLAY_BAUDRATE 115200
#define GRAPH_Y_RESOLUTION 160
#define GRAPH_MAX_TEMPERATURE 320
#define GRAPH_MAX_VOLTAGE 50
#define GRAPH_MAX_CURRENT 50
#define MAX_LOG_DISPLAY_SIZE 65
#define NUMBOX_PID_K 10000 /* constant to transform between float and int*/
#define NUMBOX_TEMP_COEF 100000 /* constant to transform between float and int*/
#define NUMBOX_R_ZERO 100/* constant to transform between float and int*/
#define WAVEFORM_X_RANGE 260

/* Variables avaiable on the display */
extern volatile uint32_t temp_sealing;
extern volatile uint32_t temp_preheat;
extern volatile uint32_t temp_measured;
extern volatile float current_rms;
extern volatile float voltage_rms;
extern volatile float pid_kp;
extern volatile float pid_ki;
extern volatile float pid_kd;
extern volatile uint32_t pid_int_limit;
extern volatile float temp_coef;
extern volatile float r_zero;
extern volatile uint16_t network_port;
extern volatile uint8_t static_ip_arr[IP_ARRAY_SIZE];
extern volatile uint16_t error_count;
extern volatile uint8_t error_log[ERROR_LOG_SIZE];
extern volatile bool flag_pot;
extern volatile uint32_t temp_measured_buffer[WAVEFORM_X_RANGE];
extern volatile float current_rms_buffer[WAVEFORM_X_RANGE];
extern volatile float voltage_rms_buffer[WAVEFORM_X_RANGE];
extern volatile uint32_t buffer_index;

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
	pg_NETWORK,
   pg_PID,
   pg_Log
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

   Overview:    Checks display events and calls associated function

   Note:     

 ********************************************************************/
void eventCheck();


/********************************************************************
   Function:    errorPage()
   Precondition: Display must be initialized
   Input:       error code
   Returns:

   Side Effects:

   Overview:    Forces error page 

   Note:     

 ********************************************************************/
void errorPage();


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
