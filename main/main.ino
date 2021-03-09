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
    -Cálculo do True RMS
    -Organizar o funcionamento do loop
    -Device driver - Ethernet
    -Device driver - Display
*/

#include "statemachine.h"
#include "statemachine.c"

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
static uint16_t BAUDRATE = 9600;

//Sensores
static double CURRENT_K1 = 29.4643; // Constante de conversão de corrente em tensão do circuito de condicionamento
static uint16_t CURRENT_K2 = 1000; // Constante de conversão do transformador de corrente
static double VOLTAGE_K1 = 41.14; // Constante do divisor resistivo do circuito de condicionamento
static double TEMP_COEF = 0.00385; // Example of temperature coefficient
static double R_ZERO = 1; //Resistance of heatband at 0ºC
volatile double current = 0; // Peak current value
volatile double voltage = 0; // Peak voltage value
volatile double currentRMS = 0; // True RMS current
volatile double voltageRMS = 0; // True RMS voltage
volatile double resistance = 0; // Vrms/Irms

//Controlo
volatile double setpoint = 0; // 0 to ~400º - Value defined by user
volatile uint16_t temp_preheat = 0; // 0 to ~400º - Value defined by user
volatile double temperature = 0; // 0 to ~400º - Calculated value from resitance
static uint16_t PID_KP = 1; // Proportional gain of PID
static uint16_t PID_KI = 1; // Integral gain of PID
//static uint16_t PID_KD = 1; // Derivative gain of PID
static uint16_t INTEGRAL_CLAMP = 1000;
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
elapsedMicros PollingTimer;
elapsedMillis SampleTimer;
elapsedMicros ZeroCrossTimer;
elapsedMillis DebounceTimer;
static uint8_t POLLING_PERIOD = 1; // Period in us
static uint8_t SAMPLING_PERIOD = 100; // Period in ms
static uint8_t DEBOUNCE_TIME = 1; // ms
volatile bool periodflag = false; // Flag used to measure period between every other zero crossing
volatile uint16_t MainsPeriod = 0; //0 to ~21000 - Period of mains to be used for True RMS

//Data EXPERIMENTAL
volatile int32_t t_data[10000];
volatile int8_t v_data[10000];

sm_t SM;

void sm_execute(sm_t *psm)
{
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
        Serial.print("\rState: ON\n");
        /*Transition actions*/
        psm->current_state = st_ON;
      }
      break;

    case st_ON:
      /* State Actions*/
      if (event == ev_ENABLE_LOW)
      {
        /*Transition actions*/
        Serial.print("\rState: OFF\n");
        psm->current_state = st_OFF;
      }
      else if ( event == ev_START_HIGH)
      {
        /*Transition actions*/
        Serial.print("\rState: CYCLESTART\n");
        psm->current_state = st_CYCLESTART;
      }
      break;

    case st_CYCLESTART:
      /* State Actions*/
      if (event == ev_START_LOW)
      {
        /*Transition actions*/
        Serial.print("\rState: ON\n");
        psm->current_state = st_ON;
      }
      else if (event == ev_PREHEAT_HIGH)
      {
        /*Transition actions*/
        Serial.print("\rState: PREHEAT\n");
        psm->current_state = st_PREHEATING;
      }
      else if ( event == ev_SEALING_HIGH)
      {
        /*Transition actions*/
        Serial.print("\rState: RAISETEMP\n");
        psm->current_state = st_RAISETEMP;
      }
      else if ( event == ev_RESET)
      {
        if (GetPinVal(EnableIO))
        {
          /*Transition actions*/
          Serial.print("\rState: ON\n");
          psm->current_state = st_ON;
        }
        else
        {
          /*Transition actions*/
          Serial.print("\rState: OFF\n");
          psm->current_state = st_OFF;
        }
      }
      break;

    case st_PREHEATING:
      /* State Actions*/
      if (event == ev_PREHEAT_LOW)
      {
        /*Transition actions*/
        Serial.print("\rState: CYCLESTART\n");
        psm->current_state = st_CYCLESTART;
      }
      else if (event == ev_SEALING_HIGH)
      {
        /*Transition actions*/
        Serial.print("\rState: RAISETEMP\n");
        psm->current_state = st_RAISETEMP;
      }
      else if ( event == ev_RESET)
      {
        if (GetPinVal(EnableIO))
        {
          /*Transition actions*/
          Serial.print("\rState: ON\n");
          psm->current_state = st_ON;
        }
        else
        {
          /*Transition actions*/
          Serial.print("\rState: OFF\n");
          psm->current_state = st_OFF;
        }
      }
      break;

    case st_RAISETEMP:
      /* State Actions*/
      if (event == ev_TEMPSET)
      {
        /*Transition actions*/
        Serial.print("\rState: SEAL\n");

        psm->current_state = st_SEAL;
      }
      else if ( event == ev_RESET)
      {
        if (GetPinVal(EnableIO))
        {
          /*Transition actions*/
          Serial.print("\rState: ON\n");
          psm->current_state = st_ON;
        }
        else
        {
          /*Transition actions*/
          Serial.print("\rState: OFF\n");
          psm->current_state = st_OFF;
        }
      }
      break;

    case st_SEAL:
      /* State Actions*/
      if (event == ev_SEALING_LOW)
      {
        /*Transition actions*/
        Serial.print("\rState: CYCLESTART\n");
        psm->current_state = st_CYCLESTART;
      }
      else if ( event == ev_RESET)
      {
        if (GetPinVal(EnableIO))
        {
          Serial.print("\rState: ON\n");
          psm->current_state = st_ON;
        }
        else
        {
          Serial.print("\rState: OFF\n");
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

/*
  Maquina de estados assíncrona: fazer excecute() a cada novo evento?
*/
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

void ZEROCROSS() {
  if (digitalRead(Zerocross) == 1 && periodflag == false)
  {
    MainsPeriod = ZeroCrossTimer;
    ZeroCrossTimer = 0;
    periodflag = true;

  }
  else if ( digitalRead(Zerocross) == 1 && periodflag == true)
  {
    periodflag = false;
  }
}

void setTemp()
{
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

void sampleCurrent()
{
  float sample = 0;
  sample = analogRead(A1Pin);
  sample = sample * 3300 / 4095;
  sample = sample - 1650;
  sample = sample / CURRENT_K1;
  sample = sample * CURRENT_K2 / 1000;
  current = sample;
  Serial.print("\rCorrente: ");
  Serial.print(sample);
  Serial.println(" Ap");
}

void sampleVoltage()
{
  float sample = 0;
  sample = analogRead(A2Pin);
  sample = sample * 3300 / 4095;
  sample = sample - 1650;
  sample = sample * VOLTAGE_K1 / 1000;
  Serial.print("\rTensao: ");
  Serial.print(sample);
  Serial.println(" Vp");
  voltage = sample;
}

void calcTemp()
{
  resistance = voltage / current;
  temperature = (resistance - R_ZERO) / (TEMP_COEF * R_ZERO);
  Serial.print("\rResistencia: ");
  Serial.print(resistance);
  Serial.println(" Ohm");
  Serial.print("\rTemperatura: ");
  Serial.print(temperature);
  Serial.println(" C");
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

  //Initialize state machine
  sm_init(&SM, st_OFF);
}

void Debounce(){
  DebounceTimer=0;
  while(DebounceTimer<DEBOUNCE_TIME);
}

bool GetPinVal(uint8_t pin){
  if(digitalRead(pin)==1)
    return true;
  return false;
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Polling
  if (PollingTimer >= POLLING_PERIOD) //Talvez seja melhor correr isto à frequência do máxima?
  {
    if (digitalRead(ResetIO) == 1 && ResetIO_old == 0)
    {
      ResetIO_old = digitalRead(ResetIO);
      RESET();
    }
    if (digitalRead(EnableIO) != EnableIO_old)
    {
      Serial.print("\rEnable\n");
      Debounce();
      EnableIO_old = digitalRead(EnableIO);
      ENABLE();
    }

    if (digitalRead(StartIO) != StartIO_old)
    {
      StartIO_old = digitalRead(StartIO);
      START();
    }

    if (digitalRead(PreheatIO) != PreheatIO_old)
    {
      PreheatIO_old = digitalRead(PreheatIO);
      PREHEAT();
    }

    if (digitalRead(SealingIO) != SealingIO_old)
    {
      SealingIO_old = digitalRead(SealingIO);
      SEALING();
    }
    PollingTimer = 0;
  }
  sm_execute(&SM);
   // Keyboard Simulation to test state machine
  uint8_t incomingByte=0;
  if (Serial.available() > 0) {
    incomingByte = Serial.read(); // Byte is received in DECIMAL format
    //Serial.print(incomingByte); 
    switch(incomingByte){
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
      default:
        sm_send_event(&SM, ev_RESET);
        sm_execute(&SM);
        break;
    }
  }
/*
  //Random tests
  if (SampleTimer >= SAMPLING_PERIOD)
  {
    Serial.print("\n\x1b[2J\r"); //Clear screen
    sampleCurrent();
    sampleVoltage();
    calcTemp();
    SampleTimer = 0;
  }*/
  /*
  if (i == 20)
  {
    Serial.print("\n\x1b[2J\r"); //Clear screen
    Serial.print("Tempo (ms): ");
    for (int j = 0; j < 20; j++) {
      Serial.print("  |  ");
      Serial.print(t_data[j]);
    }
    Serial.println();
    Serial.print("Tensao (V): ");
    for (int j = 0; j < 20; j++) {
      Serial.print("  |  ");
      Serial.print(v_data[j]);

    delay(10000);
    i=0;
  }*/
}
