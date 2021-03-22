/*
   File:   statemachine.c
   Author: Diogo Vala & Diogo Fernandes

   Overview: Define state machine
*/

#include "statemachine.h"

/*******************************************************/
void sm_init(sm_t *psm, sm_state_t initial_state)
{
  psm->current_state = initial_state;
  psm->initial_state = initial_state;
  psm->last_event = ev_NULL;
}

/*******************************************************/
void sm_send_event(sm_t *psm, sm_event_t event)
{
  psm->last_event = event;
}

/*******************************************************/
sm_state_t sm_get_current_state(sm_t *psm)
{
  return psm->current_state;
}

/*******************************************************/
 