/*
   File:   main.ino
   Author: Diogo Vala & Diogo Fernandes

   Overview: Termorregulador Digital
*/
/*TO DO
    -Terminar a função sm_execute()
      -transições para o estado de alarme
      -transição do estado de alarme para a iniciação do sistema (reset)
      -ações de cada estado
      -ações de transição
    -Device driver - Ethernet
    -Device driver - Display
    -Debouncer está com busy waiting
*/

#include "statemachine.h"
//#include "statemachine.c"
#include "utils.h"
//#include "utils.c"

/**** Analog Pins ****/
#define ANALOGpin_pot 14 // Analog input 0 - Pot
#define ANALOGpin_current 15 // Analog input 1 - Current
#define ANALOGpin_voltage 16 // Analog input 2 - Voltage

/**** Digital Pins ****/
//PLC Signals
#define IOpin_enable 2 // Digital 0 - Enable
#define IOpin_start 3 // Digital 1 - Start
#define IOpin_preheat 4 // Digital 2 - Pre-heat
#define IOpin_sealing 5 // Digital 3 - Sealing
#define IOpin_reset 6 // Digital 4 - Reset
#define IOpin_alarm 7 // Digital 5 - Alarm
//Control signals
#define CTRLpin_PWM 8 // Digital 6 - PWM output for controller
#define CTRLpin_OnOff 9 // Digital 7 - Controller On/Off
#define CTRLpin_zerocross 10 // Digital 8 - Passagem por zero


//Uart
const uint16_t BAUD_RATE = 9600;

//ADC
const uint8_t ADC_RESOLUTION = 12; // 12 bit resolution

//PWM
const uint8_t PWM_RESOLUTION = 12; // 12 bit resolution
const uint16_t MAX_DUTY_CYCLE = (1 << PWM_RESOLUTION) - 1;
const uint16_t MIN_DUTY_CYCLE = 0;

//Sensores
const float CURRENT_K1 = 29.4643; // Constante de conversão de corrente em tensão do circuito de condicionamento
const uint16_t CURRENT_K2 = 1000; // Constante de conversão do transformador de corrente
const float VOLTAGE_K1 = 41.14; // Constante do divisor resistivo do circuito de condicionamento
const float TEMP_COEF = 0.00393; // Example of temperature coefficient
const float R_ZERO = 0.8; //Resistance of heatband at reference temperature
const float T_ZERO = 1; //Reference temperature
volatile uint16_t sample_count = 0; //Number of samples taken
volatile float current = 0; // Storage of current samples
volatile float voltage = 0; // Storage of voltage samples
volatile float current_rms = 0; // True RMS current
volatile float voltage_rms = 0; // True RMS voltage
volatile float resistance = 0; // Vrms/Irms

//Controlo
volatile uint16_t temp_setpoint = 0; // 0 to ~400º - setpoint to be applied to control routine
volatile uint16_t temp_user_setpoint = 0; // 0 to ~400º - setpoint to be defined by user
volatile uint16_t temp_preheat = 0; // 0 to ~400º - Value defined by user
volatile uint16_t temp_measured = 0; // 0 to ~400º - Calculated value from resitance
volatile uint16_t temp_error_old = 0; // 0 to ~400º - Old error for derivative component
const uint16_t PID_KP = 1; // Proportional gain of PID
const uint16_t PID_KI = 1; // Integral gain of PID
const uint16_t PID_KD = 1; // Derivative gain of PID
const uint16_t INTEGRAL_CLAMP = 1000;
volatile int64_t integral = 0; // Integral component of PID
volatile int64_t derivative = 0; // Derivative component of PID
volatile uint16_t duty_cycle = 0; // 0 to 4095 - PWM duty cycle for the control signal

//Old state of input signals for polling
volatile uint8_t enable_state;
volatile uint8_t start_state;
volatile uint8_t preheat_state;
volatile uint8_t sealing_state;
volatile uint8_t reset_state;

//Timers
IntervalTimer MainTimer; // Interrupt timer
volatile unsigned long timer_polling = 0;
volatile unsigned long timer_sampling = 0;
volatile unsigned long timer_zerocross = 0;
volatile unsigned long timer_debounce = 0;
volatile unsigned long timer_print = 0;
volatile unsigned long timer_control = 0;
volatile unsigned long timer_execute_sm = 0;

//Periods
const uint32_t PERIOD_MAIN_TIMER = 1; // Period of main timer isr in us
const uint32_t PERIOD_POLLING = 1000000; //  period in us. 
const uint32_t PERIOD_SAMPLING = 1000000; //  period in us.
const uint32_t PERIOD_DEBOUNCE = 1000; // period in us. 
const uint32_t PERIOD_PRINT = 1000000; // period in us.
const uint32_t PERIOD_CONTROL = 1000000; // period in us.
const uint32_t PERIOD_SM_EXECUTE = 1000000; // period in us.

//flags
volatile bool flag_control = false; // Flag to signal that the control routine can be called
volatile bool flag_period = false; // Flag used to measure period between every other zero crossing

//State machine
sm_t state_machine;

void _timer_ISR() {
  timer_execute_sm++;
  timer_polling++;
  timer_sampling++;
  timer_zerocross++;
  timer_debounce++;
  timer_print++;
  timer_control++;
}

void sm_execute(sm_t *psm) {
  /* To do:
    -transições para o estado de alarme
    -transição do estado de alarme para a iniciação do sistema (reset)
    -ações de cada estado
    -ações de transição
  */
  sm_event_t sm_event = psm->last_event;

  switch (sm_get_current_state(psm))
  {
    /*************** OFF ***************/
    case st_OFF:
      /* State Actions*/
    if (sm_event == ev_ENABLE_HIGH)
    {
        /*Transition actions*/
      psm->current_state = st_ON;
    }
    break;

    /*************** ON ***************/
    case st_ON:
      /* State Actions*/
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
      /* State Actions*/
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
      psm->current_state = st_RAISETEMP;
    }
    else if (sm_event == ev_ENABLE_LOW)
    {
        /*Transition actions*/
      psm->current_state = st_OFF;
    }
    else if ( sm_event == ev_RESET)
    {
      if (GetPinVal(IOpin_enable))
      {
          /*Transition actions*/
        psm->current_state = st_ON;
      }
      else
      {
          /*Transition actions*/
        psm->current_state = st_OFF;
      }
    }
    break;
    /************ PREHEATING ************/
    case st_PREHEATING:
      /* State Actions*/
    if (sm_event == ev_PREHEAT_LOW)
    {
        /*Transition actions*/
      psm->current_state = st_CYCLESTART;
    }
    else if (sm_event == ev_SEALING_HIGH)
    {
        /*Transition actions*/
      psm->current_state = st_RAISETEMP;
    }
    else if (sm_event == ev_ENABLE_LOW)
    {
        /*Transition actions*/
      psm->current_state = st_OFF;
    }
    else if ( sm_event == ev_RESET)
    {
      if (GetPinVal(IOpin_enable))
      {
          /*Transition actions*/
        psm->current_state = st_ON;
      }
      else
      {
          /*Transition actions*/
        psm->current_state = st_OFF;
      }
    }
    break;
    /************ RAISETEMP ************/
    case st_RAISETEMP:
      /* State Actions*/
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
      if (GetPinVal(IOpin_enable))
      {
          /*Transition actions*/
        psm->current_state = st_ON;
      }
      else
      {
          /*Transition actions*/
        psm->current_state = st_OFF;
      }
    }
    break;

    case st_SEAL:
      /* State Actions*/
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
      if (GetPinVal(IOpin_enable))
      {
        psm->current_state = st_ON;
      }
      else
      {
        psm->current_state = st_OFF;
      }
    }
    break;
    case st_ALARM:
    break;
    default:
    break;
  }
}

// External ISR to measure the 230Vac period
void ZEROCROSS() {

  if (digitalRead(CTRLpin_zerocross) == 1 && flag_period == false)
  {
    voltage_rms = sqrt(voltage / sample_count);
    current_rms = sqrt(current / sample_count);
    calcTemp();
    sample_count = 0;
    voltage = 0;
    current = 0;
    flag_period = true;
  }
  else if ( digitalRead(CTRLpin_zerocross) == 1 && flag_period == true)
  {
    flag_period = false;
  }
}

// Control function
void setTemp(uint16_t setpoint) {
  uint16_t new_duty_cycle = duty_cycle;
  int16_t temp_error = temp_measured - setpoint;

  derivative=(temp_error-temp_error_old)/PERIOD_CONTROL;

  integral += temp_error;
  if (integral > INTEGRAL_CLAMP) integral = INTEGRAL_CLAMP; // Positive clamping to avoid wind-up
  if (integral < -INTEGRAL_CLAMP) integral = -INTEGRAL_CLAMP; // Negative clamping to avoid wind-up

  new_duty_cycle += (PID_KP * temp_error) + (PID_KI * integral) + (PID_KD * derivative); // Falta a componente derivativa

  if (new_duty_cycle > MAX_DUTY_CYCLE) {
    new_duty_cycle = MAX_DUTY_CYCLE;
  } else if (new_duty_cycle < MIN_DUTY_CYCLE) {
    new_duty_cycle = MIN_DUTY_CYCLE;
  } else {
    new_duty_cycle = new_duty_cycle;
  }
  duty_cycle = new_duty_cycle;
  analogWrite(CTRLpin_PWM, new_duty_cycle); // Sinal de controlo do controlador
}

// Samples ADC value for current and processes the data
float sampleCurrent() {
  float sample = 0;
  sample = analogRead(ANALOGpin_current);
  sample = sample * 3300 / 4095;
  sample = sample - 1650;
  sample = sample / CURRENT_K1;
  sample = sample * CURRENT_K2 / 1000;
  return sample;
}

// Samples ADC value for voltage and processes the data
float sampleVoltage() {
  float sample = 0;
  sample = analogRead(ANALOGpin_voltage);
  sample = sample * 3300 / 4095;
  sample = sample - 1650;
  sample = sample * VOLTAGE_K1 / 1000;
  return sample;
}

// Calculates the temperature at the load
void calcTemp() {
  resistance = voltage_rms / current_rms;
  temp_measured = (resistance - R_ZERO + R_ZERO * TEMP_COEF * T_ZERO) / (TEMP_COEF * R_ZERO);
}

void Debounce() {
  timer_debounce = 0;
  while (timer_debounce < PERIOD_DEBOUNCE);
}

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
  if (digitalRead(pin) == 1)
    return true;
  return false;
}

void ErrorHandler(int8_t error_code)
{
  switch(error_code){
    default:
    Serial.print("Error");
    break;
  }
}

void setup() {
  //Uart settings
  /*
    Data bits - 8
    Parity    - None
    Stop bits - 1
  */
  Serial.begin(BAUD_RATE);

  //Analog Pins
  analogReadRes(ADC_RESOLUTION); //12 bit ADC
  pinMode(ANALOGpin_pot, INPUT); // Pot
  pinMode(ANALOGpin_current, INPUT); // Current sensor
  pinMode(ANALOGpin_voltage, INPUT); // Voltage sensor

  //Interrupts
  attachInterrupt(digitalPinToInterrupt(CTRLpin_zerocross), ZEROCROSS, RISING);

  //Digital Pins
  pinMode(IOpin_enable, INPUT); // EnableIO
  pinMode(IOpin_start, INPUT); // StartIO
  pinMode(IOpin_preheat, INPUT); // Pre-heat
  pinMode(IOpin_sealing, INPUT); // SealingIO
  pinMode(IOpin_reset, INPUT); // ResetIO
  pinMode(IOpin_alarm, OUTPUT); // Alarm
  pinMode(CTRLpin_PWM, OUTPUT); // PWM output for controller
  pinMode(CTRLpin_OnOff, OUTPUT); // Controller On/Off
  pinMode(CTRLpin_zerocross, INPUT); // Passagem por zero

  //Initialize digital outputs
  digitalWrite(IOpin_alarm, LOW);
  digitalWrite(CTRLpin_PWM, LOW);
  digitalWrite(CTRLpin_OnOff, LOW);

  //PWM initialization
  analogWriteResolution(PWM_RESOLUTION); // With 12 bits, the frequency is 36621.09 Hz (teensy 4.1 doc)
  analogWrite(CTRLpin_PWM, 0); // Power controller control signal

  //Start Timer ISR
  MainTimer.begin(_timer_ISR, PERIOD_MAIN_TIMER);
  //MainTimer.priority(128); //0-255

  //Initialize state machine
  sm_init(&state_machine, st_OFF);
  Serial.print("\x1b[2J"); //Clear screen
}

void loop() {

  // Polling
  if (timer_polling >= PERIOD_POLLING) //Talvez seja melhor correr isto à frequência máxima?
  {
    // Reset pin
    if (digitalRead(IOpin_reset) == 1 && reset_state == 0)
    {
      reset_state = digitalRead(IOpin_reset);
      if (reset_state == 1)
      {
        sm_send_event(&state_machine, ev_RESET);
      }
    }
    // Enable pin
    if (digitalRead(IOpin_enable) != enable_state)
    {
      Debounce();
      enable_state = digitalRead(IOpin_enable);
      if (enable_state == 0)
      {
        sm_send_event(&state_machine, ev_ENABLE_LOW);
      }
      else
      {
        sm_send_event(&state_machine, ev_ENABLE_HIGH);
      }
    }
    // Start pin
    if (digitalRead(IOpin_start) != start_state)
    {
      Debounce();
      start_state = digitalRead(IOpin_start);
      if (start_state == 0)
      {
        sm_send_event(&state_machine, ev_START_LOW);
      }
      else
      {
        sm_send_event(&state_machine, ev_START_HIGH);
      }
    }
    // Preheat pin
    if (digitalRead(IOpin_preheat) != preheat_state)
    {
      Debounce();
      preheat_state = digitalRead(IOpin_preheat);
      if (preheat_state == 0)
      {
        sm_send_event(&state_machine, ev_PREHEAT_LOW);
      }
      else
      {
        sm_send_event(&state_machine, ev_PREHEAT_HIGH);
      }
    }
    // Sealing pin
    if (digitalRead(IOpin_sealing) != sealing_state)
    {
      Debounce();
      sealing_state = digitalRead(IOpin_sealing);
      if (sealing_state == 0)
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

  //Execute state machine
  if (timer_execute_sm >= PERIOD_SM_EXECUTE)
  {
    sm_execute(&state_machine);
    timer_execute_sm=0;
    //Serial.print("\rExecute\n");
  }
  
  //Control
  if (timer_control >= PERIOD_CONTROL && flag_control == true)
  {
    setTemp(temp_setpoint);
    timer_control = 0;
    //Serial.print("\rControl\n");
  }
  
  //Sampling
  if (timer_sampling >= PERIOD_SAMPLING)
  {
    current += power2(sampleCurrent());
    voltage += power2(sampleVoltage());
    sample_count++;
    timer_sampling = 0;
    //Serial.print("\rSampling\n");
  }
  
  //Priting
  if (timer_print >= PERIOD_PRINT) {
    Serial.print("\r\x1b[2J"); //Clear screen
    printState(&state_machine);
    Serial.print("Tensao RMS: ");
    Serial.println(voltage_rms);
    Serial.print("CorrenteRMS : ");
    Serial.println(current_rms);
    Serial.print("Temperatura: ");
    Serial.println(temp_measured);
    timer_print = 0;
  }

  // Keyboard Simulation to test state machine
  uint8_t incomingByte = 0;
  if (Serial.available() > 0) {
    incomingByte = Serial.read(); // Byte is received in DECIMAL format
    //Serial.print(incomingByte);
    switch (incomingByte) {
      case 'q':
      sm_send_event(&state_machine, ev_ENABLE_HIGH);
      sm_execute(&state_machine);
      break;
      case 'a':
      sm_send_event(&state_machine, ev_ENABLE_LOW);
      sm_execute(&state_machine);
      break;
      case 'w':
      sm_send_event(&state_machine, ev_START_HIGH);
      sm_execute(&state_machine);
      break;
      case 's':
      sm_send_event(&state_machine, ev_START_LOW);
      sm_execute(&state_machine);
      break;
      case 'e':
      sm_send_event(&state_machine, ev_PREHEAT_HIGH);
      sm_execute(&state_machine);
      break;
      case 'd':
      sm_send_event(&state_machine, ev_PREHEAT_LOW);
      sm_execute(&state_machine);
      break;
      case 'r':
      sm_send_event(&state_machine, ev_SEALING_HIGH);
      sm_execute(&state_machine);
      break;
      case 'f':
      sm_send_event(&state_machine, ev_SEALING_LOW);
      sm_execute(&state_machine);
      break;
      case 't':
      sm_send_event(&state_machine, ev_TEMPSET);
      sm_execute(&state_machine);
      break;
      default:
      sm_send_event(&state_machine, ev_RESET);
      sm_execute(&state_machine);
      break;
    }
    printState(&state_machine);
  }
}
