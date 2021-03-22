/*
   File:   statemachine.h
   Author: Diogo Vala & Diogo Fernandes

   Utility functions
*/ 

#include "utils.h"

/*******************************************************/
int GetPinVal(int pin) {
  if (digitalRead(pin) == 1)
    return 1;
  return 0;
}
/*******************************************************/

float power2(float x) {
  return x * x;
}
/*******************************************************/