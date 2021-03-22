/*
   File:   statemachine.h
   Author: Diogo Vala & Diogo Fernandes

   Utility functions
*/

#ifndef UTILS_H
#define UTILS_H

/********************************************************************
   Function:    sm_GetPinVal()
   Precondition: Pin should be configured as input
   Input:      pin number
   Returns:	   1 if pin is HIGH; 0 if pin is LOW

   Side Effects:

   Overview:     Fetches pin state

   Note:
 ********************************************************************/
int GetPinVal(int pin);

/********************************************************************
   Function:    sm_power2()
   Precondition: 
   Input:      x float
   Returns:	   x^2  

   Side Effects:

   Overview:     Calculates the x to the power of 2

   Note:
 ********************************************************************/
float power2(float x);

#endif