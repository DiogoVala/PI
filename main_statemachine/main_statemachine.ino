/* 
 * File:   main.ino
 * Author: Diogo Vala & Diogo Fernandes
 *
 * Overview: Termorregulador Digital 
 */


#include "statemachine.h"

#define A0Pin 14 // Analog input 0 - Pot
#define A1Pin 15 // Analog input 1 - Current
#define A2Pin 16 // Analog input 2 - Voltage

#define EnableIO 2 // Digital 0 - Enable
#define StartIO 3 // Digital 1 - Start
#define PreheatIO 4 // Digital 2 - Pre-heat
#define SealingIO 5 // Digital 3 - Sealing
#define ResetIO 6 // Digital 4 - Reset
#define AlarmIO 7 // Digital 5 - Alarm

#define PWMout 8 // Digital 6 - PWM output for controller
#define con_OnOff 9 // Digital 7 - Controller On/Off
#define Zeropass 10 // Digital 8 - Passagem por zero

//Uart
static uint16_t baudrate = 9600;
//Sensores
static double current_K1 = 29.4643; // Constante de conversão de corrente em tensão do circuito de condicionamento
static uint16_t current_K2 = 1000; // Constante de conversão do transformador de corrente
static double voltage_K1 = 41.14; // Constante do divisor resistivo do circuito de condicionamento
static double temp_coef=0.00385; //Example of temperature coefficient
static double R0=1; //Resistance of heatband at 0ºC
volatile double current=0; //
volatile double voltage=0; //
volatile double currentRMS=0; //
volatile double voltageRMS=0; //
volatile double resistance=0; //
//Controlo
volatile double setpoint=0; // 0 to ~400º
volatile uint16_t temp_preheat=0;  // 0 to ~400º
volatile double temperature=0; // 0 to ~400º
static uint16_t PID_Kp = 1;
static uint16_t PID_Ki = 1;
//static uint16_t PID_Kd = 1;
static uint16_t integralClamp = 100;
volatile int64_t integral = 0;
volatile int64_t derivative = 0;
volatile uint16_t duty_cycle=0; // 0 to 4095 
//Inputs
volatile uint8_t EnableIO_old;
volatile uint8_t StartIO_old;
volatile uint8_t PreheatIO_old;
volatile uint8_t SealingIO_old;
volatile uint8_t ResetIO_old;
//Timing
elapsedMillis PollingTimer;
elapsedMillis SampleTimer;
elapsedMicros ZeroCrossTimer;
static uint8_t PollingPeriod = 10; // Period in ms
static uint8_t SamplePeriod = 10; // Period in ms
volatile bool periodflag=false;
volatile uint16_t period=0; //0 to ~21000

sm_t SM;

void sm_execute(sm_t *psm)
{
  /* To do:
   *-transições para o estado de alarme
   *-transição do estado de alarme para a iniciação do sistema (reset)
   *-ações de cada estado
   *-ações de transição
  */
  sm_event_t event=psm->last_event;
  
  switch((sm_state_t)event)
  {
    case st_OFF:
      /* State Actions*/
      //if(event == ev_ENABLE_HIGH) Não funciona: 'ev_ENABLE_HIGH' was not declared in this scope
      if(event == (sm_event_t)0)
      {
        /*Transition actions*/
        psm->current_state=st_ON;
      }
      break;

    case st_ON:
      /* State Actions*/
      if(event == ev_ENABLE_LOW)
      {
        /*Transition actions*/
        psm->current_state=st_OFF;
      }
      else if( event == ev_START_HIGH)
      {
        /*Transition actions*/
        psm->current_state=st_CYCLESTART;
      }
      break;
    
    case st_CYCLESTART:
      /* State Actions*/
      if(event == ev_PREHEAT_HIGH)
      {
        /*Transition actions*/
        psm->current_state=st_PREHEATING;
      }
      else if( event == ev_SEALING_HIGH)
      {
        /*Transition actions*/
        psm->current_state=st_RAISETEMP;
      }
      else if( event == ev_RESET)
      {
        if(EnableIO == 1)
        {
          /*Transition actions*/
          psm->current_state=st_ON;
        }
        else
        {
          /*Transition actions*/
          psm->current_state=st_OFF;
        }
      }
      break;
      
    case st_PREHEATING:
      /* State Actions*/
      if(event == ev_PREHEAT_LOW)
      {
        /*Transition actions*/
        psm->current_state=st_CYCLESTART;
      }
      else if(event == ev_SEALING_HIGH)
      {
        /*Transition actions*/
        psm->current_state=st_RAISETEMP;
      }
      else if( event == ev_RESET)
      {
        if(EnableIO == 1)
        {
          /*Transition actions*/
          psm->current_state=st_ON;
        }
        else
        {
          /*Transition actions*/
          psm->current_state=st_OFF;
        }
      }
      break;
      
    case st_RAISETEMP:
      /* State Actions*/
      if(event == ev_TEMPSET)
      {
        /*Transition actions*/
        psm->current_state=st_SEAL;
      }
      else if( event == ev_RESET)
      {
        if(EnableIO == 1)
        {
          /*Transition actions*/
          psm->current_state=st_ON;
        }
        else
        {
          /*Transition actions*/
          psm->current_state=st_OFF;
        }
      }
      break;
    
    case st_SEAL:
      /* State Actions*/
      if(event == ev_SEALING_LOW)
      {
        /*Transition actions*/
        psm->current_state=st_CYCLESTART;
      }
      else if( event == ev_RESET)
      {
        if(EnableIO == 1)
        {
          /*Transition actions*/
          psm->current_state=st_ON;
        }
        else
        {
          /*Transition actions*/
          psm->current_state=st_OFF;
        }
      }
      break;
    case st_ALARM:
      break;
    default:
      break;
  }
}

void ENABLE() {
  if(digitalRead(EnableIO) == 0)
  {
    sm_send_event(&SM, ev_ENABLE_LOW);
  }
  else
  {
    sm_send_event(&SM, ev_ENABLE_HIGH);
  }
}

void START() {
  if(digitalRead(StartIO) == 0)
  {
    sm_send_event(&SM, ev_START_LOW);
  }
  else
  {
    sm_send_event(&SM, ev_START_HIGH);
  }
}

void PREHEAT() {
  if(digitalRead(PreheatIO) == 0)
  {
    sm_send_event(&SM, ev_PREHEAT_LOW);
  }
  else
  {
    sm_send_event(&SM, ev_PREHEAT_HIGH);
  }
}

void SEALING() {
  if(digitalRead(SealingIO) == 0)
  {
    sm_send_event(&SM, ev_SEALING_LOW);
  }
  else
  {
    sm_send_event(&SM, ev_SEALING_HIGH);
  }
}

void RESET() {
  if(digitalRead(ResetIO == 1))
  {
    sm_send_event(&SM, ev_RESET); 
  }
}

void ZEROPASS() {
  if(digitalRead(SealingIO) == 1 && periodflag == false)
  {
    period=ZeroCrossTimer;
    ZeroCrossTimer = 0;
    periodflag=true;
    
  }
  else if( digitalRead(SealingIO) == 1 && periodflag == true)
  {
    periodflag=false;
  }
}

void setTemp()
{
  uint16_t new_dc=duty_cycle;
  int16_t temp_error=temperature-setpoint;
  integral+=temp_error;
  if (integral > integralClamp) integral = integralClamp; // Positive clamping to avoid wind-up
  if (integral < -integralClamp) integral = -integralClamp; // Negative clamping to avoid wind-up

  
  new_dc+=(PID_Kp*temp_error)+(PID_Ki*integral); // Falta a componente derivativa

  if (new_dc > 4095) {
    new_dc = 4095;
  } else if (new_dc < 0) {
    new_dc = 0;
  } else {
    new_dc = new_dc;
  }
  analogWrite(PWMout, new_dc); // Sinal de controlo do controlador
}

void sampleCurrent()
{
  float sample=0;
  sample = analogRead(A1Pin);
  sample = sample*3300/4095; 
  sample = sample-1650;
  sample = sample/current_K1;
  sample = sample*current_K2/1000;
  current=sample;
  Serial.print("Corrente: ");
  Serial.print(sample);
  Serial.println(" Ap");
}

void sampleVoltage()
{
  float sample=0;
  sample = analogRead(A2Pin);
  sample = sample*3300/4095;
  sample = sample-1650;
  sample = sample*voltage_K1/1000;
  Serial.print("Tensao: ");
  Serial.print(sample);
  Serial.println(" Vp");
  voltage=sample;
}

void calcTemp()
{
  resistance=voltageRMS/currentRMS;
  temperature=(resistance-R0)/(temp_coef*R0);
  Serial.print("Resistencia: ");
  Serial.print(resistance);
  Serial.println(" Ohm");
  Serial.print("Temperatura: ");
  Serial.print(temperature);
  Serial.println(" C");
}

void setup() {
  //Uart settings
  /*  
   *   Data bits - 8
   *   Parity    - None
   *   Stop bits - 1
   */
  Serial.begin(baudrate);
  
  //Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(EnableIO), ENABLE, CHANGE); // EnableIO
  attachInterrupt(digitalPinToInterrupt(StartIO), START, CHANGE); // StartIO 
  attachInterrupt(digitalPinToInterrupt(PreheatIO), PREHEAT, CHANGE); // Pre-heat
  attachInterrupt(digitalPinToInterrupt(SealingIO), SEALING, CHANGE); // SealingIO
  attachInterrupt(digitalPinToInterrupt(ResetIO), RESET, RISING); // ResetIO
  attachInterrupt(digitalPinToInterrupt(Zeropass), ZEROPASS, RISING); // Passagem por zero
  
  //Analog Pins
  analogReadRes(12); //12 bit ADC
  pinMode(A0Pin, INPUT); // Pot
  pinMode(A1Pin, INPUT); // Current sensor
  pinMode(A2Pin, INPUT); // Voltage sensor
  
  //Digital Pins
  pinMode(EnableIO, INPUT); // EnableIO
  pinMode(StartIO, INPUT); // StartIO 
  pinMode(PreheatIO, INPUT); // Pre-heat
  pinMode(SealingIO, INPUT); // SealingIO
  pinMode(ResetIO, INPUT); // ResetIO 
  pinMode(AlarmIO, OUTPUT); // Alarm
  pinMode(PWMout, OUTPUT); // PWM output for controller
  pinMode(con_OnOff, OUTPUT); // Controller On/Off
  pinMode(Zeropass, INPUT); // Passagem por zero

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

void loop() {
  // put your main code here, to run repeatedly:

  if(PollingTimer>=PollingPeriod)
  {
    
    if(digitalRead(EnableIO)!=EnableIO_old)
    {
      EnableIO_old=digitalRead(EnableIO);
      ENABLE();
    }
    
    if(digitalRead(StartIO)!=StartIO_old)
    {
      StartIO_old=digitalRead(StartIO);
      START();
    }
    
    if(digitalRead(PreheatIO)!=PreheatIO_old)
    {
      PreheatIO_old=digitalRead(PreheatIO);
      PREHEAT();
    }

    if(digitalRead(SealingIO)!=SealingIO_old)
    {
      SealingIO_old=digitalRead(SealingIO);
      SEALING();
    }
    
    if(digitalRead(ResetIO)!=ResetIO_old)
    {
      ResetIO_old=digitalRead(ResetIO);
      RESET();
    }
    PollingTimer=0;
  }
/*
  while(timer1<=200);
  Serial.print("\n\x1b[2J\r"); //Clear screen
  sampleCurrent();
  sampleVoltage();
  calcTemp();
  timer1=0;*/
}
