#define A0Pin 14 // Analog input 0 - Pot
#define A1Pin 15 // Analog input 1 - Current
#define A2Pin 16 // Analog input 2 - Voltage

#define EnableIO 2 // Digital 0 - EnableIO
#define StartIO 3 // Digital 1 - StartIO 
#define PreheatIO 4 // Digital 2 - Pre-heat
#define SealingIO 5 // Digital 3 - SealingIO
#define ResetIO 6 // Digital 4 - ResetIO 
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
//Controlo
static uint16_t PID_Kp = 1;
static uint16_t PID_Ki = 1;
//static uint16_t PID_Kd = 1;
static uint16_t integralClamp = 100;

//Logica
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
  st_CYCLESTART_ISR,
  st_PREHEAT_ISRING,
  st_RAISETEMP,
  st_SEAL,
  st_ALARM
} sm_state_t;

// State machine events
typedef enum sm_event_t
{
  ev_NULL,
  ev_ENABLE_ISR_HIGH,
  ev_ENABLE_ISR_LOW,
  ev_START_ISR_HIGH,
  ev_START_ISR_LOW,
  ev_PREHEAT_ISR_HIGH,
  ev_PREHEAT_ISR_LOW,
  ev_SEALING_ISR_HIGH,
  ev_SEALING_ISR_LOW,
  ev_TEMPSET,
  ev_RESET_ISR,
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
   *-transição do estado de alarme para a iniciação do sistema (reset)
   *-ações de cada estado
   *-ações de transição
  */
  sm_event_t event=psm->last_event;
  
  switch((sm_state_t)event)
  {
    case st_OFF:
      /* State Actions*/
      if(event == ev_ENABLE_ISR_HIGH)
      {
        /*Transition actions*/
        psm->current_state=st_ON;
      }
      break;

    case st_ON:
      /* State Actions*/
      if(event == ev_ENABLE_ISR_LOW)
      {
        /*Transition actions*/
        psm->current_state=st_OFF;
      }
      else if( event == ev_START_ISR_HIGH)
      {
        /*Transition actions*/
        psm->current_state=st_CYCLESTART_ISR;
      }
      break;
    
    case st_CYCLESTART_ISR:
      /* State Actions*/
      if(event == ev_PREHEAT_ISR_HIGH)
      {
        /*Transition actions*/
        psm->current_state=st_PREHEAT_ISRING;
      }
      else if( event == ev_SEALING_ISR_HIGH)
      {
        /*Transition actions*/
        psm->current_state=st_RAISETEMP;
      }
      else if( event == ev_RESET_ISR)
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
      
    case st_PREHEAT_ISRING:
      /* State Actions*/
      if(event == ev_PREHEAT_ISR_LOW)
      {
        /*Transition actions*/
        psm->current_state=st_CYCLESTART_ISR;
      }
      else if(event == ev_SEALING_ISR_HIGH)
      {
        /*Transition actions*/
        psm->current_state=st_RAISETEMP;
      }
      else if( event == ev_RESET_ISR)
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
      else if( event == ev_RESET_ISR)
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
      if(event == ev_SEALING_ISR_LOW)
      {
        /*Transition actions*/
        psm->current_state=st_CYCLESTART_ISR;
      }
      else if( event == ev_RESET_ISR)
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

void ENABLE_ISR() {
  if(digitalRead(EnableIO) == 0)
  {
    sm_send_event(&SM, ev_ENABLE_ISR_LOW);
  }
  else
  {
    sm_send_event(&SM, ev_ENABLE_ISR_HIGH);
  }
}

void START_ISR() {
  if(digitalRead(StartIO) == 0)
  {
    sm_send_event(&SM, ev_START_ISR_LOW);
  }
  else
  {
    sm_send_event(&SM, ev_START_ISR_HIGH);
  }
}

void PREHEAT_ISR() {
  if(digitalRead(PreheatIO) == 0)
  {
    sm_send_event(&SM, ev_PREHEAT_ISR_LOW);
  }
  else
  {
    sm_send_event(&SM, ev_PREHEAT_ISR_HIGH);
  }
}

void SEALING_ISR() {
  if(digitalRead(SealingIO) == 0)
  {
    sm_send_event(&SM, ev_SEALING_ISR_LOW);
  }
  else
  {
    sm_send_event(&SM, ev_SEALING_ISR_HIGH);
  }
}

void RESET_ISR() {
  if(digitalRead(ResetIO == 1))
  {
    sm_send_event(&SM, ev_RESET_ISR); 
  }
}

elapsedMicros timer;
volatile bool periodflag=false;
void ZEROPASS_ISR() {
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
  attachInterrupt(digitalPinToInterrupt(EnableIO), ENABLE_ISR, CHANGE); // EnableIO
  attachInterrupt(digitalPinToInterrupt(StartIO), START_ISR, CHANGE); // StartIO 
  attachInterrupt(digitalPinToInterrupt(PreheatIO), PREHEAT_ISR, CHANGE); // Pre-heat
  attachInterrupt(digitalPinToInterrupt(SealingIO), SEALING_ISR, CHANGE); // SealingIO
  attachInterrupt(digitalPinToInterrupt(ResetIO), RESET_ISR, RISING); // ResetIO
  attachInterrupt(digitalPinToInterrupt(Zeropass), ZEROPASS_ISR, RISING); // Passagem por zero
  
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
  sm_init(SM, st_OFF);
}

elapsedMillis timer1;
void loop() {
  // put your main code here, to run repeatedly:
  }
  while(timer1<=200);
  Serial.print("\n\x1b[2J\r"); //Clear screen
  sampleCurrent();
  sampleVoltage();
  calcTemp();
  timer1=0;
  
}
