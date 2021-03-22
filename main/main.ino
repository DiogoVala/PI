/*
   File:   main.ino
   Author: Diogo Vala & Diogo Fernandes

   Overview: Termorregulador Digital
*/
/*TO DO
    -Incluir componente derivativa no controlador
    -Terminar a função sm_execute()
      -transições para o estado de alarme
      -transição do estado de alarme para a iniciação do sistema (reset)
      -ações de cada estado
      -ações de transição
    -Cálculo do True RMS - Primeiro teste
    -Organizar o funcionamento do loop
    -Device driver - Ethernet
    -Device driver - Display
    -Debouncer está com busy waiting
*/

#include "statemachine.h"
#include "statemachine.c"
#include "utils.h"
#include "utils.c"

/**** Analog Pins ****/
#define A0Pin 14 // Analog input 0 - Pot
#define A1Pin 15 // Analog input 1 - Current
#define A2Pin 16 // Analog input 2 - Voltage

/**** Digital Pins ****/
//PLC Signals
#define EnableIO 2 // Digital 0 - Enable
#define StartIO 3 // Digital 1 - Start
#define PreheatIO 4 // Digital 2 - Pre-heat
#define SealingIO 5 // Digital 3 - Sealing
#define ResetIO 6 // Digital 4 - Reset
#define AlarmIO 7 // Digital 5 - Alarm
//Control signals
#define PWMout 8 // Digital 6 - PWM output for controller
#define con_OnOff 9 // Digital 7 - Controller On/Off
#define Zerocross 10 // Digital 8 - Passagem por zero

//Uart
const uint16_t BAUDRATE = 9600;

//Sensores
const float CURRENT_K1 = 29.4643; // Constante de conversão de corrente em tensão do circuito de condicionamento
const uint16_t CURRENT_K2 = 1000; // Constante de conversão do transformador de corrente
const float VOLTAGE_K1 = 41.14; // Constante do divisor resistivo do circuito de condicionamento
const float TEMP_COEF = 0.00393; // Example of temperature coefficient
const float R_ZERO = 0.8; //Resistance of heatband at reference temperature
const float T_ZERO = 1; //Reference temperature
volatile float current = 0; // Storage of current samples
volatile float voltage = 0; // Storage of voltage samples
volatile uint16_t sample_count = 0; //Number of samples taken
volatile float currentRMS = 0; // True RMS current
volatile float voltageRMS = 0; // True RMS voltage
volatile float resistance = 0; // Vrms/Irms

//Controlo
volatile float setpoint = 0; // 0 to ~400º - Value defined by user
volatile uint16_t temp_preheat = 0; // 0 to ~400º - Value defined by user
volatile float temperature = 0; // 0 to ~400º - Calculated value from resitance
const uint16_t PID_KP = 1; // Proportional gain of PID
const uint16_t PID_KI = 1; // Integral gain of PID
//const uint16_t PID_KD = 1; // Derivative gain of PID
const uint16_t INTEGRAL_CLAMP = 1000;
volatile int64_t integral = 0; // Integral component of PID
volatile int64_t derivative = 0; // Derivative component of PID
volatile uint16_t duty_cycle = 0; // 0 to 4095 - PWM duty cycle for the control signal

//Old state of input signals for polling
volatile uint8_t EnableIO_old;
volatile uint8_t StartIO_old;
volatile uint8_t PreheatIO_old;
volatile uint8_t SealingIO_old;
volatile uint8_t ResetIO_old;

//Timing
IntervalTimer MainTimer; // Interrupt timer
volatile unsigned long PollingTimer = 0;
volatile unsigned long SampleTimer = 0;
volatile unsigned long ZeroCrossTimer = 0;
volatile unsigned long DebounceTimer = 0;
volatile unsigned long PrintTimer = 0;
volatile unsigned long ControlTimer = 0;
const uint32_t MAIN_TIMER_PERIOD = 1; // Period of main timer isr in us
const uint32_t POLLING_PERIOD = 10; //  period in us. Multiplies with Main_Timer_Period
const uint32_t SAMPLING_PERIOD = 100; //  period in us. Multiplies with Main_Timer_Period
const uint32_t DEBOUNCE_TIME = 1000; // period in us. Multiplies with Main_Timer_Period
const uint32_t PRINT_PERIOD = 500000; // period in us.
const uint32_t CONTROL_PERIOD = 100; // period in us.

volatile bool controlflag = false; // Flag to signal that the control routine can be called
volatile bool periodflag = false; // Flag used to measure period between every other zero crossing
volatile uint16_t MainsPeriod = 0; //0 to ~21000 - Period (us) of mains to be used for True RMS

//State machine
sm_t SM;

/*
  myTimer.begin(function, microseconds);
  myTimer.priority(number); // 0-255
  myTimer.update(microseconds); // Change the interval.
  myTimer.end(); // Stop calling the function
*/

void _timer_ISR() {
  PollingTimer++;
  SampleTimer++;
  ZeroCrossTimer++;
  DebounceTimer++;
  PrintTimer++;
}

void sm_execute(sm_t *psm) {
  /* To do:
    -transições para o estado de alarme
    -transição do estado de alarme para a iniciação do sistema (reset)
    -ações de cada estado
    -ações de transição
  */
  sm_event_t event = psm->last_event;

  switch (sm_get_current_state(psm))
  {
    case st_OFF:
      /* State Actions*/
      if (event == ev_ENABLE_HIGH)
      {
        /*Transition actions*/
        psm->current_state = st_ON;
      }
      break;

    case st_ON:
      /* State Actions*/
      if (event == ev_ENABLE_LOW)
      {
        /*Transition actions*/
        psm->current_state = st_OFF;
      }
      else if ( event == ev_START_HIGH)
      {
        /*Transition actions*/
        psm->current_state = st_CYCLESTART;
      }
      break;

    case st_CYCLESTART:
      /* State Actions*/
      if (event == ev_START_LOW)
      {
        /*Transition actions*/
        psm->current_state = st_ON;
      }
      else if (event == ev_PREHEAT_HIGH)
      {
        /*Transition actions*/
        psm->current_state = st_PREHEATING;
      }
      else if ( event == ev_SEALING_HIGH)
      {
        /*Transition actions*/
        psm->current_state = st_RAISETEMP;
      }
      else if (event == ev_ENABLE_LOW)
      {
        /*Transition actions*/
        psm->current_state = st_OFF;
      }
      else if ( event == ev_RESET)
      {
        if (GetPinVal(EnableIO))
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

    case st_PREHEATING:
      /* State Actions*/
      if (event == ev_PREHEAT_LOW)
      {
        /*Transition actions*/
        psm->current_state = st_CYCLESTART;
      }
      else if (event == ev_SEALING_HIGH)
      {
        /*Transition actions*/
        psm->current_state = st_RAISETEMP;
      }
      else if (event == ev_ENABLE_LOW)
      {
        /*Transition actions*/
        psm->current_state = st_OFF;
      }
      else if ( event == ev_RESET)
      {
        if (GetPinVal(EnableIO)==1)
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

    case st_RAISETEMP:
      /* State Actions*/
      if (event == ev_TEMPSET)
      {
        /*Transition actions*/
        psm->current_state = st_SEAL;
      }
      else if (event == ev_ENABLE_LOW)
      {
        /*Transition actions*/
        psm->current_state = st_OFF;
      }
      else if ( event == ev_RESET)
      {
        if (GetPinVal(EnableIO)==0)
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
      if (event == ev_SEALING_LOW)
      {
        /*Transition actions*/
        psm->current_state = st_CYCLESTART;
      }
      else if (event == ev_ENABLE_LOW)
      {
        /*Transition actions*/
        psm->current_state = st_OFF;
      }
      else if ( event == ev_RESET)
      {
        if (GetPinVal(EnableIO))
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

// Functions that check pin value and send new event to state machine
void ENABLE() {
  if (digitalRead(EnableIO) == 0)
  {
    sm_send_event(&SM, ev_ENABLE_LOW);
  }
  else
  {
    sm_send_event(&SM, ev_ENABLE_HIGH);
  }
}
void START() {
  if (digitalRead(StartIO) == 0)
  {
    sm_send_event(&SM, ev_START_LOW);
  }
  else
  {
    sm_send_event(&SM, ev_START_HIGH);
  }
}
void PREHEAT() {
  if (digitalRead(PreheatIO) == 0)
  {
    sm_send_event(&SM, ev_PREHEAT_LOW);
  }
  else
  {
    sm_send_event(&SM, ev_PREHEAT_HIGH);
  }
}
void SEALING() {
  if (digitalRead(SealingIO) == 0)
  {
    sm_send_event(&SM, ev_SEALING_LOW);
  }
  else
  {
    sm_send_event(&SM, ev_SEALING_HIGH);
  }
}
void RESET() {
  if (digitalRead(ResetIO == 1))
  {
    sm_send_event(&SM, ev_RESET);
  }
}

// External ISR to measure the 230Vac period
void ZEROCROSS() {
  /*
    To do:
    Periodo necessário ou basta o número de amostras em cada periodo?
  */
  if (digitalRead(Zerocross) == 1 && periodflag == false)
  {
    MainsPeriod = ZeroCrossTimer;
    ZeroCrossTimer = 0;
    voltageRMS = sqrt(voltage / sample_count);
    currentRMS = sqrt(current / sample_count);
    calcTemp();
    sample_count = 0;
    voltage = 0;
    current = 0;
    periodflag = true;
  }
  else if ( digitalRead(Zerocross) == 1 && periodflag == true)
  {
    periodflag = false;
  }
}

// Control function to generate PWM
void setTemp() {
  /* To do:
   * Componente derivativa
   */
  uint16_t new_dc = duty_cycle;
  int16_t temp_error = temperature - setpoint;
  integral += temp_error;
  if (integral > INTEGRAL_CLAMP) integral = INTEGRAL_CLAMP; // Positive clamping to avoid wind-up
  if (integral < -INTEGRAL_CLAMP) integral = -INTEGRAL_CLAMP; // Negative clamping to avoid wind-up

  new_dc += (PID_KP * temp_error) + (PID_KI * integral); // Falta a componente derivativa

  if (new_dc > 4095) {
    new_dc = 4095;
  } else if (new_dc < 0) {
    new_dc = 0;
  } else {
    new_dc = new_dc;
  }
  duty_cycle = new_dc;
  analogWrite(PWMout, new_dc); // Sinal de controlo do controlador
}

// Samples ADC value for current and processes the data
float sampleCurrent() {
  float sample = 0;
  sample = analogRead(A1Pin);
  sample = sample * 3300 / 4095;
  sample = sample - 1650;
  sample = sample / CURRENT_K1;
  sample = sample * CURRENT_K2 / 1000;
  return sample;
}

// Samples ADC value for voltage and processes the data
float sampleVoltage() {
  float sample = 0;
  sample = analogRead(A2Pin);
  sample = sample * 3300 / 4095;
  sample = sample - 1650;
  sample = sample * VOLTAGE_K1 / 1000;
  return sample;
}

// Calculates the temperature at the load
void calcTemp() {
  resistance = voltageRMS / currentRMS;
  temperature = (resistance - R_ZERO + R_ZERO * TEMP_COEF * T_ZERO) / (TEMP_COEF * R_ZERO);
}

void Debounce() {
  DebounceTimer = 0;
  while (DebounceTimer < DEBOUNCE_TIME);
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

void setup() {
  //Uart settings
  /*
    Data bits - 8
    Parity    - None
    Stop bits - 1
  */
  Serial.begin(BAUDRATE);

  //Analog Pins
  analogReadRes(12); //12 bit ADC
  pinMode(A0Pin, INPUT); // Pot
  pinMode(A1Pin, INPUT); // Current sensor
  pinMode(A2Pin, INPUT); // Voltage sensor

  //Interrupts
  attachInterrupt(digitalPinToInterrupt(Zerocross), ZEROCROSS, RISING);

  //Digital Pins
  pinMode(EnableIO, INPUT); // EnableIO
  pinMode(StartIO, INPUT); // StartIO
  pinMode(PreheatIO, INPUT); // Pre-heat
  pinMode(SealingIO, INPUT); // SealingIO
  pinMode(ResetIO, INPUT); // ResetIO
  pinMode(AlarmIO, OUTPUT); // Alarm
  pinMode(PWMout, OUTPUT); // PWM output for controller
  pinMode(con_OnOff, OUTPUT); // Controller On/Off
  pinMode(Zerocross, INPUT); // Passagem por zero

  //Initialize digital outputs
  digitalWrite(AlarmIO, LOW);
  digitalWrite(PWMout, LOW);
  digitalWrite(con_OnOff, LOW);
  //PWM initialization
  analogWriteResolution(12); // With 12 bits, the frequency is 36621.09 Hz (teensy 4.1 doc)
  analogWrite(PWMout, 0); // Power controller control signal

  //Start Timer ISR
  MainTimer.begin(_timer_ISR, MAIN_TIMER_PERIOD);
  MainTimer.priority(128);

  //Initialize state machine
  sm_init(&SM, st_OFF);
  Serial.print("\x1b[2J"); //Clear screen
}

void loop() {
  // Polling
  if (PollingTimer >= POLLING_PERIOD) //Talvez seja melhor correr isto à frequência máxima?
  {
    if (digitalRead(ResetIO) == 1 && ResetIO_old == 0)
    {
      ResetIO_old = digitalRead(ResetIO);
      RESET();
    }
    if (digitalRead(EnableIO) != EnableIO_old)
    {
      Debounce();
      EnableIO_old = digitalRead(EnableIO);
      ENABLE();
    }

    if (digitalRead(StartIO) != StartIO_old)
    {
      Debounce();
      StartIO_old = digitalRead(StartIO);
      START();
    }

    if (digitalRead(PreheatIO) != PreheatIO_old)
    {
      Debounce();
      PreheatIO_old = digitalRead(PreheatIO);
      PREHEAT();
    }

    if (digitalRead(SealingIO) != SealingIO_old)
    {
      Debounce();
      SealingIO_old = digitalRead(SealingIO);
      SEALING();
    }
    PollingTimer = 0;
  }

  //Execute state machine
  if (SampleTimer >= SAMPLING_PERIOD)
  {
    sm_execute(&SM);
  }
  
  //Control
  if (ControlTimer >= CONTROL_PERIOD && controlflag == true)
  {
    setTemp();
    ControlTimer = 0;
  }
  //Sampling
  if (SampleTimer >= SAMPLING_PERIOD)
  {
    current += power2(sampleCurrent());
    voltage += power2(sampleVoltage());
    sample_count++;
    SampleTimer = 0;
  }
  //Priting
  if (PrintTimer >= PRINT_PERIOD) {
    Serial.print("\r\x1b[2J"); //Clear screen
    printState(&SM);
    Serial.print("Tensao RMS: ");
    Serial.println(voltageRMS);
    Serial.print("CorrenteRMS : ");
    Serial.println(currentRMS);
    Serial.print("Temperatura: ");
    Serial.println(temperature);
    PrintTimer = 0;
  }

  // Keyboard Simulation to test state machine
  uint8_t incomingByte = 0;
  if (Serial.available() > 0) {
    incomingByte = Serial.read(); // Byte is received in DECIMAL format
    //Serial.print(incomingByte);
    switch (incomingByte) {
      case 'q':
        sm_send_event(&SM, ev_ENABLE_HIGH);
        sm_execute(&SM);
        break;
      case 'a':
        sm_send_event(&SM, ev_ENABLE_LOW);
        sm_execute(&SM);
        break;
      case 'w':
        sm_send_event(&SM, ev_START_HIGH);
        sm_execute(&SM);
        break;
      case 's':
        sm_send_event(&SM, ev_START_LOW);
        sm_execute(&SM);
        break;
      case 'e':
        sm_send_event(&SM, ev_PREHEAT_HIGH);
        sm_execute(&SM);
        break;
      case 'd':
        sm_send_event(&SM, ev_PREHEAT_LOW);
        sm_execute(&SM);
        break;
      case 'r':
        sm_send_event(&SM, ev_SEALING_HIGH);
        sm_execute(&SM);
        break;
      case 'f':
        sm_send_event(&SM, ev_SEALING_LOW);
        sm_execute(&SM);
        break;
      case 't':
        sm_send_event(&SM, ev_TEMPSET);
        sm_execute(&SM);
        break;
      default:
        sm_send_event(&SM, ev_RESET);
        sm_execute(&SM);
        break;
    }
  }
}
