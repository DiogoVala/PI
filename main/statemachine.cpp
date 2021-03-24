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
void sm_next_event(sm_t *psm) {

  sm_event_t sm_event = psm->last_event;

  switch (sm_get_current_state(psm))
  {
    /*************** OFF ***************/
    case st_OFF:
    if (sm_event == ev_ENABLE_HIGH)
    {
        /*Transition actions*/
      psm->current_state = st_ON;
    }
    break;

    /*************** ON ***************/
    case st_ON:
    if (sm_event == ev_ENABLE_LOW)
    {
        /*Transition actions*/
      psm->current_state = st_OFF;
    }
    else if (sm_event == ev_START_HIGH)
    {
        /*Transition actions*/
      psm->current_state = st_CYCLESTART;
    }
    break;

    /************ CYCLESTART ************/
    case st_CYCLESTART:
    if (sm_event == ev_START_LOW)
    {
        /*Transition actions*/
      psm->current_state = st_ON;
    }
    else if (sm_event == ev_PREHEAT_HIGH)
    {
        /*Transition actions*/
      psm->current_state = st_PREHEATING;
    }
    else if ( sm_event == ev_SEALING_HIGH)
    {
        /*Transition actions*/
      //psm->current_state = st_RAISETEMP;
      psm->current_state = st_SEAL;
    }
    else if (sm_event == ev_ENABLE_LOW)
    {
        /*Transition actions*/
      psm->current_state = st_OFF;
    }
    else if ( sm_event == ev_RESET)
    {
      psm->current_state = st_ON;
    }
    break;
    /************ PREHEATING ************/
    case st_PREHEATING:
    if (sm_event == ev_PREHEAT_LOW)
    {
        /*Transition actions*/
      psm->current_state = st_CYCLESTART;
    }
    else if (sm_event == ev_SEALING_HIGH)
    {
        /*Transition actions*/
      //psm->current_state = st_RAISETEMP;
    	psm->current_state = st_SEAL;
    }
    else if (sm_event == ev_ENABLE_LOW)
    {
        /*Transition actions*/
      psm->current_state = st_OFF;
    }
    else if ( sm_event == ev_RESET)
    {
      psm->current_state = st_ON;
    }
    break;
#if 0
    /************ RAISETEMP ************/
    case st_RAISETEMP:
    if (sm_event == ev_TEMPSET)
    {
        /*Transition actions*/
      psm->current_state = st_SEAL;
    }
    else if (sm_event == ev_ENABLE_LOW)
    {
        /*Transition actions*/
      psm->current_state = st_OFF;
    }
    else if ( sm_event == ev_RESET)
    {
      psm->current_state = st_ON;
    }
    break;
#endif
    /************* SEALING **************/
    case st_SEAL:
    if (sm_event == ev_SEALING_LOW)
    {
        /*Transition actions*/
      psm->current_state = st_CYCLESTART;
    }
    else if (sm_event == ev_ENABLE_LOW)
    {
        /*Transition actions*/
      psm->current_state = st_OFF;
    }
    else if ( sm_event == ev_RESET)
    {
      psm->current_state = st_ON;
    }
    break;
    /************* ALARM **************/
    case st_ALARM:
    if (sm_event == ev_ENABLE_LOW)
    {
        /*Transition actions*/
      psm->current_state = st_OFF;
    }
    else if ( sm_event == ev_RESET)
    {
      psm->current_state = st_ON;
    }
    break;
    default:
      psm->current_state = st_ALARM;
    break;
  }
}
 