/*
   File:   main.ino
   Author: Diogo Vala & Diogo Fernandes

   Overview: Termorregulador Digital
*/
/*TO DO
    -Terminar a função sm_execute()
    -Device driver - Ethernet
    -Device driver - Display
    -Debouncer está com busy waiting
    -Como fazer seleção entre potenciómetro, display e ethernet
    -Error handling
    -Optimize: Reduce division/floats
*/

#include "statemachine.h"
#include "error_codes.h"

/**** Analog Pins ****/
#define ANALOGpin_pot 14
#define ANALOGpin_current 15
#define ANALOGpin_voltage 16

/**** Digital Pins ****/
/*PLC Signals*/
#define IOpin_enable 2
#define IOpin_start 3
#define IOpin_preheat 4
#define IOpin_sealing 5
#define IOpin_reset 6
#define IOpin_alarm 7
/*Control signals*/
#define CTRLpin_PWM 8
#define CTRLpin_OnOff 9
#define CTRLpin_zerocross 10

/*General defs*/
#define LOW 0
#define HIGH 1

/*Uart*/
#define BAUD_RATE 9600

/*ADC*/
#define ADC_RESOLUTION 12

/*PWM*/
#define PWM_RESOLUTION 12
#define MAX_DUTY_CYCLE 4095 // Change according to PWM_RESOLUTION
#define MIN_DUTY_CYCLE 0

/*Sensors*/
#define CURRENT_K 29464 /* Conditioning circuit - Current to voltage conversion constant ( 29.464 * 1000 )*/
#define VOLTAGE_K 4114 /* Conditioning circuit - resistor divider constant  ( 41.14 * 100 ) */
#define TEMP_COEF 0.00393F /* Example of temperature coefficient */
#define R_ZERO 0.8F /* Resistance of heatband at reference temperature */
#define T_ZERO 20 /* Reference temperature */

/*Control*/
#define MAX_TEMP 300
#define PID_KP 1
#define PID_KI 1
#define PID_KD 1
#define INTEGRAL_CLAMP 1000

/*Periods ( in microseconds) */
#define PERIOD_MAIN 100
/* !!!These must be 2^n multiple of PERIOD_MAIN!!!*/
#define PERIOD_POLLING 800
#define PERIOD_SAMPLING 100
#define PERIOD_DEBOUNCE 1600
#define PERIOD_PRINT 100
#define PERIOD_CONTROL 1600
#define PERIOD_SM_EXECUTE 3200

/* Constants used for counting in timer ISR*/
/* Example (PERIOD_DEBOUNCE):

   if(!(timer_main&0x0F))
    timer_debounce++

  Example (PERIOD_POLLING):

   if(!(timer_main&0x07))
    timer_polling++         */
static const uint32_t COUNT_POLLING = (PERIOD_POLLING /PERIOD_MAIN)-1;
static const uint32_t COUNT_SAMPLING = (PERIOD_SAMPLING /PERIOD_MAIN)-1;
static const uint32_t COUNT_DEBOUNCE = (PERIOD_DEBOUNCE /PERIOD_MAIN)-1;
static const uint32_t COUNT_PRINT = (PERIOD_PRINT /PERIOD_MAIN)-1;
static const uint32_t COUNT_CONTROL = (PERIOD_CONTROL /PERIOD_MAIN)-1;
static const uint32_t COUNT_EXECUTE = (PERIOD_SM_EXECUTE /PERIOD_MAIN)-1;

/* Sampling */
static volatile uint8_t sample_count = 0;
static volatile int32_t sum_current = 0; /* Storage of current samples*/
static volatile int32_t sum_voltage = 0; /* Storage of voltage samples*/

/*Controlo*/
static volatile uint16_t temp_setpoint = 0; /* 0 to ~400º - internal setpoint to be applied to control routine*/
static volatile uint16_t temp_user_setpoint = 0; /* 0 to ~400º - setpoint to be defined by user*/
static volatile uint16_t temp_preheat = 0; /* 0 to ~400º - Value defined by user*/
static volatile uint16_t temp_measured = 0; /* 0 to ~400º - Calculated value from resitance*/

/*Timers*/
IntervalTimer MainTimer; /* Interrupt timer*/
static volatile uint32_t timer_polling = 0;
static volatile uint32_t timer_sampling = 0;
static volatile uint32_t timer_zerocross = 0;
static volatile uint32_t timer_debounce = 0;
static volatile uint32_t timer_print = 0;
static volatile uint32_t timer_control = 0;
static volatile uint32_t timer_execute_sm = 0;

/*flags*/
static volatile bool flag_control = false; /* Flag to signal that the control routine can be called*/
static volatile bool flag_sampling = false; /* Flag to signal that the sampling routine can be called*/
static volatile bool flag_pot_read = false; /* Flag to signal that the potentiometre can be read*/

/*State machine*/
sm_t state_machine;

static void _timer_ISR() {
  static uint32_t timer_main=0;
  timer_main++;

  /*Increment timers at 1/(2^n) times the main frequency*/
  if(!(timer_main&COUNT_POLLING))
    timer_polling++;

  if(!(timer_main&COUNT_SAMPLING))
    timer_sampling++;

  if(!(timer_main&COUNT_DEBOUNCE))
    timer_debounce++;

  if(!(timer_main&COUNT_PRINT))
    timer_print++;

  if(!(timer_main&COUNT_CONTROL))
    timer_control++;

  if(!(timer_main&COUNT_EXECUTE))
    timer_execute_sm++;
}

void sm_execute(sm_t *psm) {
  /* To do:
    -ações de cada estado
  */
  Serial.print("\r\x1b[2J");
  printState(psm);
  switch (sm_get_current_state(psm))
  {
    /*************** OFF ***************/
    case st_OFF:
      /* State Actions*/
    digitalWrite(IOpin_alarm, LOW);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_sampling=false;
    flag_control=false;

    Serial.print("\rSampling: OFF\n");
    Serial.print("\rTemp. Control: OFF\n");
    break;

    /*************** ON ***************/
    case st_ON:
        /* State Actions*/
    digitalWrite(IOpin_alarm, HIGH);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_sampling=true;
    flag_control=false;

    Serial.print("\rSampling: ON\n");
    Serial.print("\rTemp. Control: OFF\n");
    break;

    /************ CYCLESTART ************/
    case st_CYCLESTART:
      /* State Actions*/
    digitalWrite(IOpin_alarm, HIGH);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_sampling=true;
    flag_control=false;

    Serial.print("\rSampling: ON\n");
    Serial.print("\rTemp. Control: OFF\n");
    break;

    /************ PREHEATING ************/
    case st_PREHEATING:
      /* State Actions*/
    digitalWrite(IOpin_alarm, HIGH);
    digitalWrite(CTRLpin_OnOff, HIGH);
    flag_sampling=true;
    flag_control=true;
    temp_setpoint=temp_preheat;

    Serial.print("\rSampling: ON\n");
    Serial.print("\rTemp. Control: ON\n");
    break;

    /************* SEALING **************/
    case st_SEAL:
      /* State Actions*/
    digitalWrite(IOpin_alarm, HIGH);
    digitalWrite(CTRLpin_OnOff, HIGH);
    flag_sampling=true;
    flag_control=true;
    temp_setpoint=temp_user_setpoint;

    Serial.print("\rSampling: ON\n");
    Serial.print("\rTemp. Control: ON\n");
    break;

    /************* ALARM **************/
    case st_ALARM:
      /* State Actions*/
    digitalWrite(IOpin_alarm, LOW);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_sampling=false;
    flag_control=false;
    Serial.print("\rSampling: OFF\n");
    Serial.print("\rTemp. Control: OFF\n");
    break;
    /* Note: Maybe put ALARM inside default case */
    default:
      /* State Actions*/
    digitalWrite(IOpin_alarm, LOW);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_sampling=false;
    flag_control=false;
    break;
  }
}

/* External input ISR to calculate RMS and temperature */
static void ZEROCROSS() {

  static float current_rms = 0;
  static float voltage_rms = 0;
  static float resistance_rms = 0;
  static bool flag_period = false; /* Flag used to measure period between every other zero crossing*/

  /* T=R*1/(TEMP_COEF*R_ZERO)+(1/TEMP_COEF-T_ZERO)*/
  static const uint16_t T_slope=1/(TEMP_COEF*R_ZERO);
  static const uint16_t T_b=1/TEMP_COEF-T_ZERO;

  if (digitalRead(CTRLpin_zerocross) == HIGH && flag_period == false)
  {
    voltage_rms = sqrt(sum_voltage / sample_count);
    current_rms = sqrt(sum_current / sample_count);
    resistance_rms = voltage_rms / current_rms;
    /*temp_measured = (resistance - R_ZERO + R_ZERO * TEMP_COEF * T_ZERO) / (TEMP_COEF * R_ZERO);*/
    temp_measured = resistance_rms*T_slope-T_b;
    if(temp_measured>MAX_TEMP)
    {
      errorHandler(MAX_TEMP_EXCEEDED);
    }

    sample_count = 0;
    sum_voltage = 0;
    sum_current = 0;
    flag_period = true;
  }
  else if ( digitalRead(CTRLpin_zerocross) == HIGH && flag_period == true)
  {
    flag_period = false;
  }
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

static float sampleCurrent() {

  int32_t sample = 0;
  sample = analogRead(ANALOGpin_current);
  sample = (sample * 33000000)>>ADC_RESOLUTION; /* [0 , 33000000] V */
  sample = sample - 16500000; /* [-16500000 , 16500000] V */
  sample = sample / CURRENT_K; /* [-16500000 , 16500000] / 2946 = [-5600 , 5600] Amps*100 */
  return sample;
}

static float sampleVoltage() {
  int32_t sample = 0;;
  sample = analogRead(ANALOGpin_voltage);
  sample = (sample * 330000)>>ADC_RESOLUTION; /* [0 , 330000] V */
  sample = sample - 165000; /* [-165000 , 165000] V */
  sample = sample * VOLTAGE_K / 100000; /* [-165000 , 165000] V  * 4114 / 100000 = [-6788 , 6788] V*100*/
  return sample;
}

static void debounce() {
  timer_debounce = 0;
  while (timer_debounce < PERIOD_DEBOUNCE);
}

/*Function not needed in final product*/
static void printState(sm_t *psm) {
  Serial.print("\n\rState: ");
  switch (sm_get_current_state(psm)) {
    case st_OFF:
    Serial.println("OFF");
    break;
    case st_ON:
    Serial.println("ON");
    break;
    case st_CYCLESTART:
    Serial.println("CYCLESTART");
    break;
    case st_PREHEATING:
    Serial.println("PREHEATING");
    break;
    case st_RAISETEMP:
    Serial.println("RAISETEMP");
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
  switch(error_code){
    default:
    Serial.print("Error");
    break;
  }
}

void setup() {
  /*Uart settings
    Data bits - 8
    Parity    - None
    Stop bits - 1
  */
  Serial.begin(BAUD_RATE);

  /*Analog Pins*/
  analogReadRes(ADC_RESOLUTION);
  pinMode(ANALOGpin_pot, INPUT);
  pinMode(ANALOGpin_current, INPUT);
  pinMode(ANALOGpin_voltage, INPUT);

  /*Interrupts*/
  attachInterrupt(digitalPinToInterrupt(CTRLpin_zerocross), ZEROCROSS, RISING);

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
  analogWrite(CTRLpin_PWM, LOW); /* Power controller control signal*/

  /*Start Timer ISR*/
  MainTimer.begin(_timer_ISR, PERIOD_MAIN);

  /*Initialize state machine*/
  sm_init(&state_machine, st_OFF);

  Serial.print("\x1b[2J"); /*Clear screen*/
}

void loop() {

  /*Old state of input signals for polling*/
  static uint8_t enable_state;
  static uint8_t start_state;
  static uint8_t preheat_state;
  static uint8_t sealing_state;
  static uint8_t reset_state;

  /* Polling*/
  if (timer_polling >= PERIOD_MAIN)
  {
    /* Reset pin*/
    if (digitalRead(IOpin_reset) == HIGH && reset_state == LOW)
    {
      reset_state = digitalRead(IOpin_reset);
      if (reset_state == HIGH)
      {
        sm_send_event(&state_machine, ev_RESET);
      }
    }
    /* Enable pin*/
    if (digitalRead(IOpin_enable) != enable_state)
    {
      debounce();
      enable_state = digitalRead(IOpin_enable);
      if (enable_state == LOW)
      {
        sm_send_event(&state_machine, ev_ENABLE_LOW);
      }
      else
      {
        sm_send_event(&state_machine, ev_ENABLE_HIGH);
      }
    }
    /* Start pin*/
    if (digitalRead(IOpin_start) != start_state)
    {
      debounce();
      start_state = digitalRead(IOpin_start);
      if (start_state == LOW)
      {
        sm_send_event(&state_machine, ev_START_LOW);
      }
      else
      {
        sm_send_event(&state_machine, ev_START_HIGH);
      }
    }
    /* Preheat pin*/
    if (digitalRead(IOpin_preheat) != preheat_state)
    {
      debounce();
      preheat_state = digitalRead(IOpin_preheat);
      if (preheat_state == LOW)
      {
        sm_send_event(&state_machine, ev_PREHEAT_LOW);
      }
      else
      {
        sm_send_event(&state_machine, ev_PREHEAT_HIGH);
      }
    }
    /* Sealing pin*/
    if (digitalRead(IOpin_sealing) != sealing_state)
    {
      debounce();
      sealing_state = digitalRead(IOpin_sealing);
      if (sealing_state == LOW)
      {
        sm_send_event(&state_machine, ev_SEALING_LOW);
      }
      else
      {
        sm_send_event(&state_machine, ev_SEALING_HIGH);
      }
    }
    timer_polling = 0;
  }

#if 0
  /*Priting*/
  if (timer_print >= PERIOD_PRINT) {
    Serial.print("\r\x1b[2J"); /*Clear screen*/
    printState(&state_machine);
    Serial.print("Tensao RMS: ");
    Serial.println(voltage_rms);
    Serial.print("CorrenteRMS : ");
    Serial.println(current_rms);
    Serial.print("Temperatura: ");
    Serial.println(temp_measured);
    timer_print = 0;
  }
#endif

  /*Execute state machine*/
  if (timer_execute_sm >= PERIOD_MAIN)
  {
    sm_next_event(&state_machine);
    sm_execute(&state_machine);
    timer_execute_sm=0;
    /*Serial.print("\rExecute\n");*/
  }

  /*Sampling*/
  if (timer_sampling >= PERIOD_MAIN && flag_sampling == true)
  {
    if(flag_pot_read == true)
    {
      temp_user_setpoint=(analogRead(ANALOGpin_pot)*MAX_TEMP)>>ADC_RESOLUTION ; /* Read pot value*/
    }
    /*Note: How to handle critial sections?*/
    /*Serial.println(temp_user_setpoint);*/
    uint32_t temp;
    temp=sampleCurrent();
    sum_current += temp*temp;

    temp=sampleVoltage();
    sum_voltage += temp*temp;

    sample_count++;
    timer_sampling = 0;

    //Serial.print("\rSampling\n");
  }

  /*Control*/
  if (timer_control >= PERIOD_MAIN && flag_control == true)
  {
    controlTemp(temp_setpoint);
    timer_control = 0;
    //Serial.print("\rControl\n");
  }
  else if (timer_control >= PERIOD_MAIN && flag_control == false)
  {
    analogWrite(CTRLpin_PWM, 0); /*XXX: Might be MAX_DUTY_CYCLE if logic is inverted*/
  }

  /* Keyboard Simulation to test state machine*/
  uint8_t incomingByte = 0;
  if (Serial.available() > 0) {
    incomingByte = Serial.read(); /* Byte is received in DECIMAL format*/
    /*Serial.print(incomingByte);*/
    switch (incomingByte) {
      case 'q':
      sm_send_event(&state_machine, ev_ENABLE_HIGH);
      break;
      case 'a':
      sm_send_event(&state_machine, ev_ENABLE_LOW);
      break;
      case 'w':
      sm_send_event(&state_machine, ev_START_HIGH);
      break;
      case 's':
      sm_send_event(&state_machine, ev_START_LOW);
      break;
      case 'e':
      sm_send_event(&state_machine, ev_PREHEAT_HIGH);
      break;
      case 'd':
      sm_send_event(&state_machine, ev_PREHEAT_LOW);
      break;
      case 'r':
      sm_send_event(&state_machine, ev_SEALING_HIGH);
      break;
      case 'f':
      sm_send_event(&state_machine, ev_SEALING_LOW);
      break;
      case 't':
      sm_send_event(&state_machine, ev_TEMPSET);
      break;
      default:
      sm_send_event(&state_machine, ev_RESET);
      break;
    }
    sm_next_event(&state_machine);
    sm_execute(&state_machine);
    //printState(&state_machine);
  }
}
