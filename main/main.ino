/*
   File:   main.ino
   Author: Diogo Vala & Diogo Fernandes

   Overview: Termorregulador Digital
*/
/*TO DO
    -Device driver - Ethernet
    -Device driver - Display
    -Debouncer está com busy waiting
    -Como fazer seleção entre potenciómetro, display e ethernet
    -Error handling
    -Optimize: Reduce division/floats
*/

/*
 * Tested and working:
 *  - PWM working at 2000. Controller saturates because there's no feedback.
 *  - Polling working and states change accordingly.
 *  - Temperature calculation working
 *  - State machine working with external signals
 */


#include "statemachine.h"
#include "ethernet.h"
#include "DisplayDriver.h"
#include "error_handler.h"
#include "config.h"

#define DEBUGGING 0
#define ERRORCHECKING 0

static const uint32_t COUNT_POLLING = (PERIOD_POLLING / PERIOD_MAIN);
static const uint32_t COUNT_SAMPLING = (PERIOD_SAMPLING / PERIOD_MAIN);
static const uint32_t COUNT_CONTROL = (PERIOD_CONTROL / PERIOD_MAIN);
static const uint32_t COUNT_EXECUTE = (PERIOD_SM_EXECUTE / PERIOD_MAIN);
static const uint32_t COUNT_DISPLAY = (PERIOD_DISPLAY / PERIOD_MAIN);

/* Sampling */
static volatile uint8_t sample_count = 0;
static volatile int64_t sum_current = 0; /* Storage of current samples*/
static volatile int64_t sum_voltage = 0; /* Storage of voltage samples*/

/*Controlo*/
static volatile uint32_t temp_setpoint = 0; /* 0 to ~400º - internal setpoint to be applied to control routine*/
volatile uint32_t temp_sealing = 0; /* 0 to ~400º - setpoint to be defined by user*/
volatile uint32_t temp_preheat = 0; /* 0 to ~400º - Value defined by user*/
volatile uint32_t temp_measured = 0; /* 0 to ~400º - Calculated value from resitance*/
volatile float current_rms = 0;
volatile float voltage_rms = 0;

/*Timers*/
IntervalTimer Timer_Main; /* Interrupt timer*/
IntervalTimer Timer_230V;
static volatile uint32_t count_polling = 0;
static volatile uint32_t count_sampling = 0;
static volatile uint32_t count_control = 0;
static volatile uint32_t count_execute_sm = 0;
static volatile uint32_t count_display = 0;

/*flags*/
static volatile bool flag_control = false; /* Flag to signal that the control routine can be called*/
static volatile bool flag_sampling = false; /* Flag to signal that the sampling routine can be called*/
static volatile bool flag_pot_read = false; /* Flag to signal that the potentiometre can be read*/
static volatile bool flag_execute = true;

/*State machines*/
static sm_t main_machine;
static sm_t sub_machine;

static void _timer_ISR() {
  count_polling++;
  count_sampling++;
  count_control++;
  count_execute_sm++;
  count_display++;
}

static void sm_execute_main(sm_t *psm) {

#if DEBUGGING
  Serial.print("\r\x1b[2J"); /*Clear screen*/
  Serial.print("\rTermorregulador Digital\n");
  Serial.print("\rPreheat temp.: ");
  Serial.println(temp_preheat);
  Serial.print("\rSealing temp.: ");
  Serial.println(temp_sealing);
  Serial.print("\rMeasured temp: ");
  Serial.println(temp_measured);
  Serial.print("\rVoltage RMS: ");
  Serial.println(voltage_rms/100);
  Serial.print("\rCurrent RMS: ");
  Serial.println(current_rms/100);
  Serial.print("\rState: ");
  printState(psm);
#endif

  switch (sm_get_current_state(psm))
  {
    /*************** OFF ***************/
    case st_OFF:
    /*State Actions*/
    digitalWrite(IOpin_alarm, LOW);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_sampling=false;
    flag_control=false;

    /*Transitions*/
    if (psm->last_event == ev_ENABLE_HIGH)
    {
      psm->current_state = st_ON;
    }
    break;

    /*************** ON ***************/
    case st_ON:
    /*State Actions*/
    flag_sampling=true;

    /*Transitions*/
    if (psm->last_event == ev_ENABLE_LOW)
    {
      psm->current_state = st_OFF;
      sm_init(&sub_machine, st_IDLE);
    }
    if (psm->last_event == ev_RESET)
    {
      resetDisplay();
      sm_init(&sub_machine, st_IDLE);
      psm->last_event = ev_NULL;
    }

    sm_execute_sub(&sub_machine);
    break;

    /************* ALARM **************/
    case st_ALARM:
    /*State Actions*/
    digitalWrite(IOpin_alarm, LOW);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_sampling=false;
    flag_control=false;

    /*Transitions*/
    if (psm->last_event == ev_ENABLE_LOW)
    {
      resetDisplay();
      psm->current_state = st_OFF;
      sm_init(&sub_machine, st_IDLE);
    }
    if (psm->last_event == ev_RESET)
    {
      resetDisplay();
      psm->current_state = st_ON;
      sm_init(&sub_machine, st_IDLE);
    }
    break;
  }
  /*Force Alarm state */
  if(psm->last_event == ev_OK_LOW)
  {
    psm->current_state = st_ALARM;
  }
}

static void sm_execute_sub(sm_t *psm){

#if DEBUGGING
  Serial.print("\rSubstate: ");
  printState(psm);
#endif
  switch(sm_get_current_state(psm))
  {
  /*************** IDLE ***************/
    case st_IDLE:
    /*State Actions*/
    digitalWrite(IOpin_alarm, HIGH);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_control=false;

    /*Transitions*/
    if (psm->last_event == ev_START_HIGH)
    {
      psm->current_state = st_CYCLESTART;
    }
    break;

    /************ CYCLESTART ************/
    case st_CYCLESTART:
    /*State Actions*/
    digitalWrite(IOpin_alarm, HIGH);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_control=false;

    /*Transitions*/
    if (psm->last_event == ev_START_LOW)
    {
      psm->current_state = st_IDLE;
    }
    else if (psm->last_event == ev_PREHEAT_HIGH)
    {
      psm->current_state = st_PREHEATING;
    }
    else if ( psm->last_event == ev_SEALING_HIGH)
    {
      psm->current_state = st_SEAL;
    }
    break;

    /************ PREHEATING ************/
    case st_PREHEATING:
    /*State Actions*/
    digitalWrite(IOpin_alarm, HIGH);
    digitalWrite(CTRLpin_OnOff, HIGH);
    flag_control=true;

    temp_setpoint=temp_preheat;

    /*Transitions*/
    if (psm->last_event == ev_PREHEAT_LOW)
    {
      psm->current_state = st_CYCLESTART;
    }
    else if (psm->last_event == ev_SEALING_HIGH)
    {
      psm->current_state = st_SEAL;
    }
    break;

    /************* SEALING **************/
    case st_SEAL:
    /*State Actions*/
    digitalWrite(IOpin_alarm, HIGH);
    digitalWrite(CTRLpin_OnOff, HIGH);
    flag_control=true;

    temp_setpoint=temp_sealing;

    /*Transitions*/
    if (psm->last_event == ev_SEALING_LOW)
    {
      psm->current_state = st_CYCLESTART;
    }
    break;
  }
}

/* 20ms-periodic to calculate RMS and temperature */
static void _calcTemp_ISR() {

  static float resistance_rms = 0;

  /* T=R*1/(TEMP_COEF*R_ZERO)+(1/TEMP_COEF-T_ZERO)*/
  static const float T_slope=1/(TEMP_COEF*R_ZERO);
  static const float T_b=1/TEMP_COEF-T_ZERO;

  voltage_rms = sqrt(sum_voltage / sample_count);
  if(voltage_rms/100>MAX_VOLTAGE)
  {
    errorHandler(ERROR_MAX_VOLTAGE_EXCEEDED);
  }
  else if(voltage_rms/100<5)
  {
    voltage_rms=1;
  }

  current_rms = sqrt(sum_current / sample_count);
  if(current_rms/100>MAX_CURRENT)
  {
    errorHandler(ERROR_MAX_CURRENT_EXCEEDED);
  }
  else if(current_rms/100<5)
  {
    current_rms=1;
  }

  resistance_rms = voltage_rms / current_rms;

  /*temp_measured = (resistance - R_ZERO + R_ZERO * TEMP_COEF * T_ZERO) / (TEMP_COEF * R_ZERO);*/
  temp_measured = (resistance_rms*T_slope-T_b);

  if(temp_measured>MAX_TEMPERATURE)
  {
    errorHandler(ERROR_MAX_TEMPERATURE_EXCEEDED);
  }

  sample_count = 0;
  sum_voltage = 0;
  sum_current = 0;
}

static void controlTemp(uint16_t setpoint) {

  static uint16_t temp_error_old = 0; /* 0 to ~400º - Old error for derivative component*/
  static int32_t integral = 0; /* Integral component of PID*/
  static int32_t derivative = 0; /* Derivative component of PID*/
  static uint16_t duty_cycle = 0; /* 0 to 4095 - PWM duty cycle for the control signal*/

  uint16_t new_duty_cycle = duty_cycle;
  int16_t temp_error = temp_measured - setpoint;

  /*Controlo*/

  derivative=(temp_error-temp_error_old);

  integral += temp_error;
  if (integral > INTEGRAL_CLAMP) integral = INTEGRAL_CLAMP;
  if (integral < -INTEGRAL_CLAMP) integral = -INTEGRAL_CLAMP;

  Serial.print("Derivative: ");
  Serial.println(derivative);
  Serial.print("Integral: ");
  Serial.println(integral);
  Serial.print("Proportional: ");
  Serial.println(temp_error);

  new_duty_cycle += ((PID_KP * temp_error) + (PID_KI * integral) + (PID_KD * derivative));

  if (new_duty_cycle > MAX_DUTY_CYCLE) {
    new_duty_cycle = MAX_DUTY_CYCLE;
  } 
  else if (new_duty_cycle < MIN_DUTY_CYCLE) {
    new_duty_cycle = MIN_DUTY_CYCLE;
  }

  duty_cycle = new_duty_cycle;
  analogWrite(CTRLpin_PWM, new_duty_cycle); /* Sinal de controlo do controlador*/
}

static int32_t sampleCurrent() {

  int32_t sample = 0;
  sample = analogRead(ANALOGpin_current);
  sample = (sample * 330000)>>ADC_RESOLUTION; /* [0 , 330000] V */
  sample = sample - 165000; /* [-165000 , 165000] V */
  sample = sample*1000 / CURRENT_K; /* [-165000 , 165000] / 29464 = [-5600 , 5600] Amps*100 */
  return sample;
}

static int32_t sampleVoltage() {
  int32_t sample = 0;
  sample = analogRead(ANALOGpin_voltage);
  sample = (sample * 330000)>>ADC_RESOLUTION; /* [0 , 330000] V */
  sample = sample - 165000; /* [-165000 , 165000] V */
  sample = sample * VOLTAGE_K / 100000; /* [-165000 , 165000] V  * 4114 / 100000 = [-6788 , 6788] V*100 */
  return sample;
}

/*Function not needed in final product*/
static void printState(sm_t *psm) {
  switch (sm_get_current_state(psm)) {
    case st_OFF:
    Serial.println("OFF");
    break;
    case st_ON:
    Serial.println("ON");
    break;
    case st_IDLE:
    Serial.println("IDLE");
    break;
    case st_CYCLESTART:
    Serial.println("CYCLESTART");
    break;
    case st_PREHEATING:
    Serial.println("PREHEATING");
    break;
    case st_SEAL:
    Serial.println("SEAL");
    break;
    case st_ALARM:
    Serial.println("ALARM");
    break;
  }
}

static void errorHandler(int8_t error_code){
#if ERRORCHECKING
  errorPage(error_code);
  sm_send_event(&main_machine, ev_OK_LOW);
#endif
}

void pauseSystem(){
  flag_execute = false;
  flag_control = false;
  flag_sampling = false;
  flag_pot_read = false;
}

void unpauseSystem(){
  main_machine.current_state=st_ON;
  sub_machine.current_state=st_IDLE;
  flag_execute = true;
}

void setup() {
  /*Uart settings
    Data bits - 8
    Parity    - None
    Stop bits - 1
  */
  Serial.begin(UART_BAUDRATE);
  Serial.print("\x1b[2J"); /*Clear screen*/
  Serial.println("Starting...");
  /*Analog Pins*/
  analogReadRes(ADC_RESOLUTION);
  //pinMode(ANALOGpin_pot, INPUT);
  //pinMode(ANALOGpin_current, INPUT);
  //pinMode(ANALOGpin_voltage, INPUT);

  /*Digital Pins*/
  pinMode(IOpin_enable, INPUT);
  pinMode(IOpin_start, INPUT);
  pinMode(IOpin_preheat, INPUT);
  pinMode(IOpin_sealing, INPUT);
  pinMode(IOpin_reset, INPUT);
  pinMode(IOpin_alarm, OUTPUT);
  pinMode(CTRLpin_PWM, OUTPUT);
  pinMode(CTRLpin_OnOff, OUTPUT);
  pinMode(CTRLpin_zerocross, INPUT);

  /*Initialize digital outputs*/
  digitalWrite(IOpin_alarm, LOW);
  digitalWrite(CTRLpin_PWM, LOW);
  digitalWrite(CTRLpin_OnOff, LOW);

  /*PWM initialization*/
  analogWriteResolution(PWM_RESOLUTION); /* With 12 bits, the frequency is 36621.09 Hz (teensy 4.1 doc)*/
  analogWriteFrequency(CTRLpin_PWM, PWM_FREQUENCY);
  analogWrite(CTRLpin_PWM, LOW); /* Power controller control signal*/

  /*Initialize ethernet module and API*/
 //InitEthernet();
  InitDisplay();

  /*Start Timer ISR*/
  Timer_Main.begin(_timer_ISR, PERIOD_MAIN);
  Timer_230V.begin(_calcTemp_ISR, PERIOD_230V);

  /*Initialize state machine*/
  sm_init(&main_machine, st_OFF);
  sm_init(&sub_machine, st_IDLE);
  Serial.println("Done.");
  //
}

void loop() {
  ListenClient();
  
  /*Old states of input signals for polling*/
  static uint8_t enable_state = LOW;
  static uint8_t start_state = LOW;
  static uint8_t preheat_state = LOW;
  static uint8_t sealing_state = LOW;
  static uint8_t reset_state = LOW;

  /* Polling*/
  if (count_polling >= COUNT_POLLING)
  {
    eventCheck(); /*Display Polling*/

    /* Reset pin*/
    if (digitalRead(IOpin_reset) != reset_state)
    {
      reset_state = digitalRead(IOpin_reset);
      if (reset_state == HIGH)
      {
        sm_send_event(&main_machine, ev_RESET);
      }
    }
    /* Enable pin*/
    if (digitalRead(IOpin_enable) != enable_state)
    {
      enable_state = digitalRead(IOpin_enable);
      if (enable_state == LOW)
      {
        sm_send_event(&main_machine, ev_ENABLE_LOW);
      }
      else
      {
        sm_send_event(&main_machine, ev_ENABLE_HIGH);
      }
    }
    /* Start pin*/
    if (digitalRead(IOpin_start) != start_state)
    {
      //debounce();
      start_state = digitalRead(IOpin_start);
      if (start_state == LOW)
      {
        sm_send_event(&sub_machine, ev_START_LOW);
      }
      else
      {
        sm_send_event(&sub_machine, ev_START_HIGH);
      }
    }
    /* Preheat pin*/
    if (digitalRead(IOpin_preheat) != preheat_state)
    {
      //debounce();
      preheat_state = digitalRead(IOpin_preheat);
      if (preheat_state == LOW)
      {
        sm_send_event(&sub_machine, ev_PREHEAT_LOW);
      }
      else
      {
        sm_send_event(&sub_machine, ev_PREHEAT_HIGH);
      }
    }
    /* Sealing pin*/
    if (digitalRead(IOpin_sealing) != sealing_state)
    {
      //debounce();
      sealing_state = digitalRead(IOpin_sealing);
      if (sealing_state == LOW)
      {
        sm_send_event(&sub_machine, ev_SEALING_LOW);
      }
      else
      {
        sm_send_event(&sub_machine, ev_SEALING_HIGH);
      }
    }
    count_polling = 0;
  }

  /*Execute state machine*/
  if (count_execute_sm >= COUNT_EXECUTE && flag_execute == true)
  {
    sm_execute_main(&main_machine);
    count_execute_sm=0;
    /*Serial.print("\rExecute\n");*/
  }

  /*Sampling*/
  if (count_sampling >= COUNT_SAMPLING && flag_sampling == true)
  {
    if(flag_pot_read == true)
    {
      temp_sealing=(analogRead(ANALOGpin_pot)*MAX_TEMPERATURE)>>ADC_RESOLUTION ; /* Read pot value*/
    }
    /*Serial.println(temp_sealing);*/
    int32_t sample;
    sample=sampleCurrent();
    sum_current += sample*sample;

    sample=sampleVoltage();
    sum_voltage += sample*sample;

    sample_count++;

    count_sampling = 0;
  }

  /*Control*/
  if (count_control >= COUNT_CONTROL && flag_control == true)
  {
    controlTemp(temp_setpoint);
    count_control = 0;
  }
  else if (count_control >= COUNT_CONTROL && flag_control == false)
  {
    analogWrite(CTRLpin_PWM, MAX_DUTY_CYCLE/2); /*XXX: Might be MAX_DUTY_CYCLE if logic is inverted*/
  }
  
  /*Display*/
  if(count_display >= COUNT_DISPLAY)
  {
    updateDisplay(sub_machine.current_state, start_state, preheat_state, sealing_state);
    count_display = 0;
  }

  /* Keyboard Simulation to test state machine*/
  uint8_t incomingByte = 0;
  if (Serial.available() > 0) {
    incomingByte = Serial.read(); /* Byte is received in DECIMAL format*/
    switch (incomingByte) {
      case 'q':
      sm_send_event(&main_machine, ev_ENABLE_HIGH);
      break;
      case 'a':
      sm_send_event(&main_machine, ev_ENABLE_LOW);
      break;
      case 'w':
      sm_send_event(&sub_machine, ev_START_HIGH);
      break;
      case 's':
      sm_send_event(&sub_machine, ev_START_LOW);
      break;
      case 'e':
      sm_send_event(&sub_machine, ev_PREHEAT_HIGH);
      break;
      case 'd':
      sm_send_event(&sub_machine, ev_PREHEAT_LOW);
      break;
      case 'r':
      sm_send_event(&sub_machine, ev_SEALING_HIGH);
      break;
      case 'f':
      sm_send_event(&sub_machine, ev_SEALING_LOW);
      break;
      case 't':
      sm_send_event(&main_machine, ev_RESET);
      break;
    }
  }
}
