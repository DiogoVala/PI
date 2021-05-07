/*
   File:   statemachine.h
   Author: Diogo Vala & Diogo Fernandes

   Overview: Define state machines
*/

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

//State machine states
typedef enum {
  /*Main Machine*/
  st_OFF,
  st_ON,
  st_ALARM,
  
  /*Sub machine*/
  st_IDLE,
  st_CYCLESTART,
  st_PREHEATING,
  st_SEAL
} sm_state_t;

//State machine events
typedef enum {
  ev_NULL,

  /*Main Machine*/
  ev_ENABLE_HIGH,
  ev_ENABLE_LOW,
  ev_RESET,
  ev_OK_LOW,

  /*Sub machine*/
  ev_START_HIGH,
  ev_START_LOW,
  ev_PREHEAT_HIGH,
  ev_PREHEAT_LOW,
  ev_SEALING_HIGH,
  ev_SEALING_LOW,
} sm_event_t;

//State machine definition
typedef struct{
  sm_state_t current_state;
  sm_state_t initial_state;
  sm_event_t last_event;
} sm_t;

/********************************************************************
   Function:    sm_init()
   Precondition: Define sm_state_t and sm_event_t structs
   Input:      *psm (Pointer to sm_t) ; initial_state
   Returns:

   Side Effects:

   Overview:     Initiates the state machine at initial_state

   Note:     last_event is set to ev_NULL

 ********************************************************************/
void sm_init(sm_t *psm, sm_state_t initial_state);

/********************************************************************
   Function:    sm_send_event()
   Precondition: states and events should be defined;
                 state machine should be initiated
   Input:      *psm (Pointer to sm_t) ; event that occurred
   Returns:

   Side Effects:

   Overview:     Sets last_event of sm_t to event

   Note:

 ********************************************************************/
void sm_send_event(sm_t *psm, sm_event_t event);

/********************************************************************
   Function:    sm_get_current_state()
   Precondition:

   Input:      *psm (Pointer to sm_t)
   Returns:    current_state of sm_t

   Side Effects:

   Overview:

   Note:

 ********************************************************************/
sm_state_t sm_get_current_state(sm_t *psm);

/********************************************************************
   Function:    sm_execute()
   Precondition: states and events should be defined;
                 state machine should be initiated

   Input:      *psm (Pointer to sm_t)
   Returns:

   Side Effects:

   Overview:   Runs tasks inside states

   Note:       Should be manually added in main file

 ********************************************************************/
void sm_execute(sm_t *psm);
 
/********************************************************************
   Function:    sm_next_event()
   Precondition: states and events should be defined;
                 state machine should be initiated

   Input:      *psm (Pointer to sm_t)
   Returns:

   Side Effects:

   Overview:   Handles events and changes states

   Note:       

 ********************************************************************/
void sm_next_event(sm_t *psm);

#endif
