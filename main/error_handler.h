/*
   File:   statemachine.h
   Author: Diogo Vala & Diogo Fernandes

   Overview: Error codes
*/

#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

/* Error codes for the system */
#define ERROR_CODE_SIZE 3
#define ERROR_LOG_SIZE 3900

#define ERROR_MAX_TEMPERATURE_EXCEEDED 100
#define ERROR_MAX_VOLTAGE_EXCEEDED 101
#define ERROR_MAX_CURRENT_EXCEEDED 102
#define ERROR_MAX_RESISTANCE_EXCEEDED 103
#define ERROR_CURRENT_HIGHER_THAN_VOLTAGE 104
#define ERROR_RESISTANCE_LOWER_THAN_RZERO 105
#define ERROR_ZERO_TEMPERATURE 106

#endif