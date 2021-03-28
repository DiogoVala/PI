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
    -Optimize variables (reduce number of floats)
*/

#include "statemachine.h"

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
#define MAX_DUTY_CYCLE 4095
#define MIN_DUTY_CYCLE 0

/*Sensores*/
#define CURRENT_K 29464 /* Constante de conversão de corrente em tensão do circuito de condicionamento ( 29.464*1000 )*/
#define VOLTAGE_K 4114 /* Constante do divisor resistivo do circuito de condicionamento (41.14*100)*/
#define TEMP_COEF 0.00393F /* Example of temperature coefficient*/
#define R_ZERO 0.8F /*Resistance of heatband at reference temperature*/
#define T_ZERO 20 /*Reference temperature*/

/*Controlo*/
#define MAX_TEMP 300
#define PID_KP 1
#define PID_KI 1
#define PID_KD 1
#define INTEGRAL_CLAMP 1000

/*Periods ( in microseconds) */
const uint32_t PERIOD_MAIN = 100;
/* 2^n multiples of Main timer*/
const uint32_t PERIOD_POLLING = (800 /PERIOD_MAIN)-1;
const uint32_t PERIOD_SAMPLING = (100 /PERIOD_MAIN)-1;
const uint32_t PERIOD_DEBOUNCE = (1600 /PERIOD_MAIN)-1;
const uint32_t PERIOD_PRINT = (100 /PERIOD_MAIN)-1;
const uint32_t PERIOD_CONTROL = (1600 /PERIOD_MAIN)-1;
const uint32_t PERIOD_SM_EXECUTE = (3200 /PERIOD_MAIN)-1;

/* T=R*1/(TEMP_COEF*R_ZERO)+(1/TEMP_COEF-T_ZERO)*/
const uint16_t T_slope=1/(TEMP_COEF*R_ZERO);
const uint16_t T_b=1/TEMP_COEF-T_ZERO;

volatile uint16_t sample_count = 0; /*Number of samples taken*/
volatile int32_t current = 0; /* Storage of current samples*/
volatile int32_t voltage = 0; /* Storage of voltage samples*/
volatile float current_rms = 0; /* True RMS current*/
volatile float voltage_rms = 0; /* True RMS voltage*/
volatile float resistance = 0; /* Vrms/Irms*/

/*Controlo*/
volatile uint16_t temp_setpoint = 0; /* 0 to ~400º - setpoint to be applied to control routine*/
volatile uint16_t temp_user_setpoint = 0; /* 0 to ~400º - setpoint to be defined by user*/
volatile uint16_t temp_preheat = 0; /* 0 to ~400º - Value defined by user*/
volatile uint16_t temp_measured = 0; /* 0 to ~400º - Calculated value from resitance*/
volatile uint16_t temp_error_old = 0; /* 0 to ~400º - Old error for derivative component*/
volatile int64_t integral = 0; /* Integral component of PID*/
volatile int32_t derivative = 0; /* Derivative component of PID*/
volatile uint16_t duty_cycle = 0; /* 0 to 4095 - PWM duty cycle for the control signal*/

/*Old state of input signals for polling*/
volatile uint8_t enable_state;
volatile uint8_t start_state;
volatile uint8_t preheat_state;
volatile uint8_t sealing_state;
volatile uint8_t reset_state;

/*Timers*/
IntervalTimer MainTimer; /* Interrupt timer*/
static uint32_t timer_main=0;
volatile uint32_t timer_polling = 0;
volatile uint32_t timer_sampling = 0;
volatile uint32_t timer_zerocross = 0;
volatile uint32_t timer_debounce = 0;
volatile uint32_t timer_print = 0;
volatile uint32_t timer_control = 0;
volatile uint32_t timer_execute_sm = 0;

/*flags*/
volatile bool flag_control = false; /* Flag to signal that the control routine can be called*/
volatile bool flag_sampling = false; /* Flag to signal that the sampling routine can be called*/
volatile bool flag_pot_read = false; /* Flag to signal that the potentiometre can be read*/
volatile bool flag_period = false; /* Flag used to measure period between every other zero crossing*/


/*State machine*/
sm_t state_machine;

void _timer_ISR() {
  timer_main++;
  /*Increment timers at 1/(2^n) times the main frequency*/
  if(!(timer_main&PERIOD_POLLING))
    timer_polling++;
  if(!(timer_main&PERIOD_SAMPLING))
    timer_sampling++;
  if(!(timer_main&PERIOD_DEBOUNCE))
    timer_debounce++;
  if(!(timer_main&PERIOD_PRINT))
    timer_print++;
  if(!(timer_main&PERIOD_CONTROL))
    timer_control++;
  if(!(timer_main&PERIOD_SM_EXECUTE))
    timer_execute_sm++;
}

void sm_execute(sm_t *psm) {
  /* To do:
    -ações de cada estado
  */
  switch (sm_get_current_state(psm))
  {
    /*************** OFF ***************/
    case st_OFF:
      /* State Actions*/
    digitalWrite(IOpin_alarm, LOW);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_sampling=false;
    flag_control=false;
    break;

    /*************** ON ***************/
    case st_ON:
        /* State Actions*/
    digitalWrite(IOpin_alarm, HIGH);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_sampling=true;
    flag_control=false;
    break;

    /************ CYCLESTART ************/
    case st_CYCLESTART:
      /* State Actions*/
    digitalWrite(IOpin_alarm, HIGH);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_sampling=true;
    flag_control=false;
    break;

    /************ PREHEATING ************/
    case st_PREHEATING:
      /* State Actions*/
    digitalWrite(IOpin_alarm, HIGH);
    digitalWrite(CTRLpin_OnOff, HIGH);
    flag_sampling=true;
    flag_control=true;
    temp_setpoint=temp_preheat;
    break;

    /************* SEALING **************/
    case st_SEAL:
      /* State Actions*/
    digitalWrite(IOpin_alarm, HIGH);
    digitalWrite(CTRLpin_OnOff, HIGH);
    flag_sampling=true;
    flag_control=true;
    temp_setpoint=temp_user_setpoint;
    break;

    /************* ALARM **************/
    case st_ALARM:
      /* State Actions*/
    digitalWrite(IOpin_alarm, LOW);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_sampling=false;
    flag_control=false;
    break;
    default:
      /* State Actions*/
    digitalWrite(IOpin_alarm, LOW);
    digitalWrite(CTRLpin_OnOff, LOW);
    flag_sampling=false;
    flag_control=false;
    break;
  }
}

/* External ISR to measure the 230Vac period*/
void ZEROCROSS() {

  if (digitalRead(CTRLpin_zerocross) == HIGH && flag_period == false)
  {
    voltage_rms = sqrt(voltage / sample_count);
    current_rms = sqrt(current / sample_count);
    resistance = voltage_rms / current_rms;
    /*temp_measured = (resistance - R_ZERO + R_ZERO * TEMP_COEF * T_ZERO) / (TEMP_COEF * R_ZERO);*/
    temp_measured = resistance*T_slope-T_b;
    sample_count = 0;
    voltage = 0;
    current = 0;
    flag_period = true;
  }
  else if ( digitalRead(CTRLpin_zerocross) == HIGH && flag_period == true)
  {
    flag_period = false;
  }
}

/* Control function*/
void setTemp(uint16_t setpoint) {
  uint16_t new_duty_cycle = duty_cycle;
  int16_t temp_error = temp_measured - setpoint;

  derivative=(temp_error-temp_error_old);

  integral += temp_error;
  if (integral > INTEGRAL_CLAMP) integral = INTEGRAL_CLAMP;
  if (integral < -INTEGRAL_CLAMP) integral = -INTEGRAL_CLAMP;

  new_duty_cycle += ((PID_KP * temp_error) + (PID_KI * integral) + (PID_KD * derivative));

  if (new_duty_cycle > MAX_DUTY_CYCLE) {
    new_duty_cycle = MAX_DUTY_CYCLE;
  } else if (new_duty_cycle < MIN_DUTY_CYCLE) {
    new_duty_cycle = MIN_DUTY_CYCLE;
  } else {
    new_duty_cycle = new_duty_cycle;
  }
  duty_cycle = new_duty_cycle;
  analogWrite(CTRLpin_PWM, new_duty_cycle); /* Sinal de controlo do controlador*/
}

/* Samples ADC value for current and processes the data*/
float sampleCurrent() {

  int32_t sample = 0;
  sample = analogRead(ANALOGpin_current);
  sample = (sample * 33000000)>>ADC_RESOLUTION; /* [0 , 33000000] V */
  sample = sample - 16500000; /* [-16500000 , 16500000] V */
  sample = sample / CURRENT_K; /* [-16500000 , 16500000] / 2946 = [-5600 , 5600] Amps*100 */
  return sample;
}

/* Samples ADC value for voltage and processes the data*/
float sampleVoltage() {
  int32_t sample = 0;;
  sample = analogRead(ANALOGpin_voltage);
  sample = (sample * 330000)>>ADC_RESOLUTION; /* [0 , 330000] V */
  sample = sample - 165000; /* [-165000 , 165000] V */
  sample = sample * VOLTAGE_K / 100000; /* [-165000 , 165000] V  * 4114 / 100000 = [-6788 , 6788] V*100*/
  return sample;
}

void Debounce() {
  timer_debounce = 0;
  while (timer_debounce < PERIOD_DEBOUNCE);
}

/*Function not needed in final product*/
void printState(sm_t *psm) {
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

bool GetPinVal(int pin) {
  if (digitalRead(pin) == HIGH)
    return true;
  return false;
}

void ErrorHandler(int8_t error_code){
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
      Debounce();
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
      Debounce();
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
      Debounce();
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
      Debounce();
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
    current += temp*temp;
    temp=sampleVoltage();
    voltage += temp*temp;
    sample_count++;
    timer_sampling = 0;

    Serial.print("\rSampling\n");
  }

  /*Control*/
  if (timer_control >= PERIOD_MAIN && flag_control == true)
  {
    setTemp(temp_setpoint);
    timer_control = 0;
    Serial.print("\rControl\n");
  }
  else if (timer_control >= PERIOD_MAIN && flag_control == false)
  {
    analogWrite(CTRLpin_PWM, 0); /*Might be MAX_DUTY_CYCLE if logic is inverted: Set controller to minimum power*/
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
    printState(&state_machine);
  }
}
