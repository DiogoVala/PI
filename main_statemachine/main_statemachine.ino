#define A0Pin 14 // Analog input 0 - Pot
#define A1Pin 15 // Analog input 1 - Current
#define A2Pin 16 // Analog input 2 - Voltage

#define EnableIO 2 // Digital 0 - EnableIO
#define StartIO 3 // Digital 1 - StartIO 
#define PreheatIO 4 // Digital 2 - Pre-heat
#define SealingIO 5 // Digital 3 - SealingIO
#define ResetIO 6 // Digital 4 - ResetIO 
#define AlarmIO 7 // Digital 5 - Alarm

#define D6Pin 8 // Digital 6 - PWM output for controller
#define D7Pin 9 // Digital 7 - Controller On/Off
#define Zeropass 10 // Digital 8 - Passagem por zero

//Uart
static uint16_t baudrate = 9600;
//Sensores
static double current_K1 = 29.4643; // Constante de conversão de corrente em tensão do circuito de condicionamento
static uint16_t current_K2 = 1000; // Constante de conversão do transformador de corrente
static double voltage_K1 = 41.14; // Constante do divisor resistivo do circuito de condicionamento
//Controlo
static uint16_t PID_Kp = 1;
static uint16_t PID_Ki = 1;
//static uint16_t PID_Kd = 1;
static uint16_t integralClamp = 100;

//Logica
volatile uint8_t state = 0;
/*  0 - Sistema Desligado
 *  1 - Sistema Ligado
 *  2 - Inicio de ciclo
 *  3 - Pre-heat
 *  4 - Aumentar Temp
 *  5 - Selagem
 *  6 - Alarme
*/
volatile uint16_t setpoint=0; // 0 to ~400º
volatile uint16_t temp_preheat=0;  // 0 to ~400º
volatile uint16_t temp=0; // 0 to ~400º
volatile uint16_t period=0; //0 to ~21000
volatile uint16_t duty=0; // 0 to 4095
//Sensores
volatile double current=0;
volatile double voltage=0;
//Controlo
volatile int64_t integral = 0;
volatile int64_t derivative = 0;
volatile uint16_t dc=0;

// State machine states
typedef enum sm_state_t
{
  st_OFF,
  st_ON,
  st_CYCLESTART,
  st_PREHEATING,
  st_RAISETEMP,
  st_SEAL,
  st_ALARM
} sm_state_t;

// State machine events
typedef enum sm_event_t
{
  ev_NULL,
  ev_ENABLE_HIGH,
  ev_ENABLE_LOW,
  ev_START_HIGH,
  ev_START_LOW,
  ev_PREHEAT_HIGH,
  ev_PREHEAT_LOW,
  ev_SEALING_HIGH,
  ev_SEALING_LOW,
  ev_TEMPSET,
  ev_RESET,
  ev_ALARM

} sm_event_t;

//State machine definition
#define sm_state_t int
typedef struct sm_t
{
  sm_state_t current_state;
  sm_state_t initial_state;
  sm_event_t last_event;
} sm_t;

sm_t SM; // State Machine declaration
/*
void sm_init(sm_t *psm, sm_state_t initial_state);
void sm_reset(sm_t *psm);
sm_state_t sm_get_current_state(sm_t *psm);
void sm_send_event(sm_t *psm, sm_event_t event);
void sm_execute_AB(sm_t *psm);
void sm_execute_XYZ(sm_t *psm);
*/

void sm_init(sm_t *psm, sm_state_t initial_state)
{
  psm->current_state=initial_state;
  psm->initial_state=initial_state;
  psm->last_event=ev_NULL;
}
void sm_send_event(sm_t *psm, sm_event_t event)
{
  psm->last_event=event;
}

sm_state_t sm_get_current_state(sm_t *psm)
{
  return psm->current_state;
}

void sm_execute(sm_t *psm)
{
  /* To do:
   *-transições para o estado de alarme
   *-transição do estado de alarme para a iniciação do sistema
   *-ações de cada estado
   *-ações de transição
  */
  sm_event_t event=psm->last_event;
  switch((sm_state_t)event)
  {
    case st_OFF:
      /* State Actions*/
      if(event == ev_ENABLE_HIGH)
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
        /*Transition actions*/
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
    state = 0; // Sistema desligado
  }
  else
  {
    sm_send_event(&SM, ev_ENABLE_HIGH);
    state = 1; // Sistema ligado
  }
}

void START() {
  if(digitalRead(StartIO) == 0 && state == 2)
  {
    state = 1; // Sistema Ligado
  }
  else if( digitalRead(StartIO) == 1 && state == 1)
  {
    state = 2; // Inicio de ciclo
  }
  else
  {
    state = 6; // Alarme
  }
}

void PREHEAT() {
  if(digitalRead(PreheatIO) == 0 && state == 2)
  {
    state = 2; // Inicio de ciclo
  }
  else if( digitalRead(PreheatIO) == 1 && state == 2)
  {
    state = 3; // Pre-heat
  }
  else
  {
    state = 6; // Alarme
  }
}

void SEALING() {
  if(digitalRead(SealingIO) == 0 && state == 2)
  {
    state = 2; // Inicio de ciclo
  }
  else if( digitalRead(SealingIO) == 1 && ( state == 2 || state ==3))
  {
    state = 4; // Aumentar Temp
  }
  else if( digitalRead(SealingIO) == 0 && state == 5)
  {
    state = 2; // Inicio de ciclo
  }
  else
  {
    state = 6; // Alarme
  }
}

void RESET() {
  if(digitalRead(ResetIO == 1))
  {
    state = 0;  
  }
}

elapsedMicros timer;
volatile bool periodflag=false;
void ZEROPASS() {
  if(digitalRead(SealingIO) == 1 && periodflag == false)
  {
    period=timer;
    timer = 0;
    periodflag=true;
    
  }
  else if( digitalRead(SealingIO) == 1 && periodflag == true)
  {
    periodflag=false;
  }
}

void turnOff()
{
}

void turnOn()
{
}

void standby()
{
}

void preheat()
{
  
}

void setTemp()
{
  uint16_t new_dc=dc;
  int16_t temp_error=temp-setpoint;
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
  analogWrite(D6Pin, new_dc); // Sinal de controlo do controlador
}

void alarme()
{
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
}


void setup() {
  Serial.begin(baudrate);
  analogReadRes(12); //12 bit ADC

  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(EnableIO), ENABLE, CHANGE); // EnableIO
  attachInterrupt(digitalPinToInterrupt(StartIO), START, CHANGE); // StartIO 
  attachInterrupt(digitalPinToInterrupt(PreheatIO), PREHEAT, CHANGE); // Pre-heat
  attachInterrupt(digitalPinToInterrupt(SealingIO), SEALING, CHANGE); // SealingIO
  attachInterrupt(digitalPinToInterrupt(ResetIO), RESET, RISING); // ResetIO
  attachInterrupt(digitalPinToInterrupt(Zeropass), ZEROPASS, RISING); // Passagem por zero
  pinMode(A0Pin, INPUT); // Pot
  pinMode(A1Pin, INPUT); // Current sensor
  pinMode(A2Pin, INPUT); // Voltage sensor
  pinMode(EnableIO, INPUT); // EnableIO
  pinMode(StartIO, INPUT); // StartIO 
  pinMode(PreheatIO, INPUT); // Pre-heat
  pinMode(SealingIO, INPUT); // SealingIO
  pinMode(ResetIO, INPUT); // ResetIO 
  pinMode(AlarmIO, OUTPUT); // Alarm
  pinMode(D6Pin, OUTPUT); // PWM output for controller
  pinMode(D7Pin, OUTPUT); // Controller On/Off
  pinMode(Zeropass, INPUT); // Passagem por zero
  digitalWrite(AlarmIO, LOW);
  digitalWrite(D6Pin, LOW);
  digitalWrite(D7Pin, LOW);
  analogWriteResolution(12); // 2-15bits | A 12 bits, a frequencia e 36621.09 Hz (teensy 4.1 doc)
  analogWrite(D6Pin, 0); // Sinal de controlo do controlador
}

elapsedMillis timer1;
void loop() {
  // put your main code here, to run repeatedly:
  //noInterrupts();
  switch(state)
  {
    case 0:
      turnOff();
      break;
    case 1:
      turnOn();
      break;
    case 2:
      standby();
      break;
    case 3:
      preheat();
      break;
    case 4:
      setTemp();
      break;
    case 5:
      // acender led para mostrar que se atingiu a temp. pretendida e que está a selar
      break;
    default:
      alarme();
      break;
  }
  while(timer1<=200);
  Serial.print("\n\x1b[2J\r"); //Clear screen
  sampleCurrent();
  sampleVoltage();
  calcTemp();
  timer1=0;
  
}
