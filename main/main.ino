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
#include "EEPROM_utils.h"

#define DEBUGGING 0
#define ERRORCHECKING 1

static const uint32_t COUNT_POLLING = (PERIOD_POLLING / PERIOD_MAIN);
static const uint32_t COUNT_SAMPLING = (PERIOD_SAMPLING / PERIOD_MAIN);
static const uint32_t COUNT_CONTROL = (PERIOD_CONTROL / PERIOD_MAIN);
static const uint32_t COUNT_EXECUTE = (PERIOD_SM_EXECUTE / PERIOD_MAIN);
static const uint32_t COUNT_DISPLAY = (PERIOD_DISPLAY / PERIOD_MAIN);

static const uint32_t SAMPLES_PER_PERIOD = PERIOD_230V/PERIOD_SAMPLING;
static volatile uint32_t calibration_current_sensor = 0;
static volatile uint32_t calibration_voltage_sensor = 0;

/* Sampling */
static volatile uint8_t sample_count = 0; /* Counts samples taken */
static volatile int64_t sum_current = 0; /* Storage of current samples*/
static volatile int64_t sum_voltage = 0; /* Storage of voltage samples*/

/*Control*/
static volatile uint32_t temp_setpoint = 0; /* Internal setpoint to be applied to control routine*/
static volatile uint32_t temp_pot = 0;
volatile uint32_t temp_sealing = 0; /* Value defined by user*/
volatile uint32_t temp_preheat = 100; /* Value defined by user*/
volatile uint32_t temp_measured = 0; /* Calculated value from resistance */
volatile float current_rms = 0;
volatile float voltage_rms = 0;
volatile double duty_cycle = 0; /* 0 to 4095 - PWM duty cycle for the control signal*/
volatile float pid_kp=0.2; // 0.2
volatile float pid_ki=0.1; // 0.1
volatile float pid_kd=0.001; // 0.001 
volatile uint32_t pid_int_limit=10; //10
volatile float temp_coef = 0.001;
volatile float r_zero = 1.01;
extern volatile uint16_t network_port;
extern volatile uint8_t static_ip_arr[IP_ARRAY_SIZE];

/*Error Logging*/
volatile uint16_t error_count = 0;
volatile uint8_t error_log[ERROR_LOG_SIZE]={0};

/*Hardware timer*/
IntervalTimer Timer_Main; /* Interrupt timer*/

/*Timekeeping counters*/
static volatile uint32_t count_polling = 0;
static volatile uint32_t count_sampling = 0;
static volatile uint32_t count_control = 0;
static volatile uint32_t count_execute_sm = 0;
static volatile uint32_t count_display = 0;

/*flags*/
static volatile bool flag_control = false; /* Flag to signal that the control routine can be called*/
static volatile bool flag_sampling = false; /* Flag to signal that the sampling routine can be called*/
static volatile bool flag_execute = true; /* Flag to signal that the sm_execute routine can be called*/

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

    sample_count = 0;
    sum_current = 0;
    sum_voltage = 0;
    duty_cycle=0;
    temp_measured = 0;
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
      //sysReset();
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

static void sm_execute_sub(sm_t *psm) {
#if DEBBUGGING
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

static void controlTemp(uint16_t setpoint) {

  static int32_t temp_error_old = 0; /* 0 to ~400º - Old error for derivative component*/
  static double integral = 0; /* Integral component of PID*/
  static double derivative = 0; /* Derivative component of PID*/
  static double proportional = 0;

  int32_t new_duty_cycle = duty_cycle;
  int32_t temp_error = setpoint - temp_measured;

  proportional=temp_error;

  /*Controlo*/
  derivative=(temp_error-temp_error_old)/((float)PERIOD_CONTROL/1000000);

  integral += temp_error;
  if (integral >  pid_int_limit) integral =  pid_int_limit;
  if (integral < -pid_int_limit) integral = -pid_int_limit;

  new_duty_cycle += ((proportional*pid_kp) + (integral*pid_ki) + (derivative*pid_kd));

  if (new_duty_cycle > MAX_DUTY_CYCLE) {
    new_duty_cycle = MAX_DUTY_CYCLE;
  } 
  else if (new_duty_cycle < MIN_DUTY_CYCLE) {
    new_duty_cycle = MIN_DUTY_CYCLE;
  }
//Serial.println(new_duty_cycle);

#if 0
  Serial.print("\n");
  Serial.print("Proportional: ");
  Serial.println(temp_error/pid_kp);
  Serial.print("Integral: ");
  Serial.println(integral/pid_ki);
  Serial.print("Derivative: ");
  Serial.println(derivative/pid_kd);
  Serial.print("DC: ");
  Serial.println(new_duty_cycle);
  Serial.print("Temp: ");
  Serial.println(temp_measured);
#endif

  duty_cycle = new_duty_cycle;
  
  analogWrite(CTRLpin_PWM, new_duty_cycle); /* Sinal de controlo do controlador*/
}

static int32_t sampleCurrent() {
  int32_t sample = 0;
  sample = analogRead(ANALOGpin_current);
  sample = (sample * CURRENT_M)-CURRENT_B+calibration_current_sensor;
  
  if (abs(sample) < MIN_SENSOR_VAL && flag_control == true)
  {
    sample=0;
  }
  
  return sample;
}

static int32_t sampleVoltage() {
  int32_t sample = 0;
  sample = analogRead(ANALOGpin_voltage);
  sample = (sample * VOLTAGE_M)-VOLTAGE_B+calibration_voltage_sensor;
 //Serial.println(sample);
  if (abs(sample) < MIN_SENSOR_VAL && flag_control == true)
  {
    sample=0;
  }
  //Serial.println(sample);
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

static void errorHandler(int16_t error_code) {
#if ERRORCHECKING
  static int16_t last_error = 0;
  if(last_error != error_code)
  {
    writeInt16ToEEPROM(ADDR_ERROR_COUNT, error_count);
    writeInt8ToEEPROM(ADDR_ERROR_LOG+error_count, error_code);
    error_log[error_count]=error_code;
    error_count++;
    if(error_count>ERROR_LOG_SIZE)
    {
      error_count=0;
    }
    //errorPage(error_code);
    last_error=error_code;
    //sm_send_event(&main_machine, ev_OK_LOW);
  }
#endif
}

void pauseSystem() {
  flag_execute = false;
  flag_control = false;
  flag_sampling = false;
}

void unpauseSystem() {
  main_machine.current_state=st_ON;
  sub_machine.current_state=st_IDLE;
  flag_execute = true;
}

void sysReset() {
  sample_count = 0;
  sum_current = 0;
  sum_voltage = 0;
  duty_cycle=0;
  sm_init(&sub_machine, st_IDLE);
  sm_init(&main_machine, st_ON);
  resetDisplay();
}

void loadMemory(){

  temp_preheat = readInt32FromEEPROM(ADDR_TEMP_PREHEAT);
  temp_sealing = readInt32FromEEPROM(ADDR_TEMP_SEALING);
  pid_kp = (float)readInt32FromEEPROM(ADDR_PID_KP)/NUMBOX_PID_K;
  pid_ki =(float)readInt32FromEEPROM(ADDR_PID_KI)/NUMBOX_PID_K;
  pid_kd = (float)readInt32FromEEPROM(ADDR_PID_KD)/NUMBOX_PID_K;
  pid_int_limit = readInt32FromEEPROM(ADDR_PID_INT_LIM);
  temp_coef = (float)readInt32FromEEPROM(ADDR_TEMP_COEF)/NUMBOX_TEMP_COEF;
  r_zero = (float)readInt16FromEEPROM(ADDR_R_ZERO)/NUMBOX_R_ZERO;
  network_port = readInt16FromEEPROM(ADDR_NETWORK_PORT);
  error_count = readInt16FromEEPROM(ADDR_ERROR_COUNT);
  Serial.println(error_count);
  if(error_count>ERROR_LOG_SIZE)
  {
    error_count=ERROR_LOG_SIZE;
  }
  for (uint16_t i = 0; i<error_count; i++)
  {
    error_log[i]=readInt8FromEEPROM(ADDR_ERROR_LOG+i);
    Serial.println(error_log[i]);
  }

  for (uint8_t i = 0; i<IP_ARRAY_SIZE; i++)
  {
    static_ip_arr[i] = readInt8FromEEPROM(ADDR_STATIC_IP+i);
  }
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

  loadMemory();

  /*Analog Pins*/
  analogReadRes(ADC_RESOLUTION);

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
  analogWriteFrequency(CTRLpin_PWM, PWM_FREQUENCY_HZ);
  analogWrite(CTRLpin_PWM, LOW); /* Power controller control signal*/

  /*Initialize ethernet module and API*/
 //InitEthernet();
  InitDisplay();

  /*Start Timer ISR*/
  Timer_Main.begin(_timer_ISR, PERIOD_MAIN);

  /*Initialize state machine*/
  sm_init(&main_machine, st_OFF);
  sm_init(&sub_machine, st_IDLE);
  Serial.println("Running.");
}

void loop() {
  ListenClient();
  eventCheck(); /*Display Polling*/

  /*Old states of input signals for polling*/
  static uint8_t enable_state = LOW;
  static uint8_t start_state = LOW;
  static uint8_t preheat_state = LOW;
  static uint8_t sealing_state = LOW;
  static uint8_t reset_state = LOW;

  /*Variables for temperature calculation*/
  static float resistance_rms = 0;
  static const float T_slope=1/(TEMP_COEF*R_ZERO);
  static const float T_b=1/TEMP_COEF-T_ZERO;

  /* Polling*/
  if (count_polling >= COUNT_POLLING)
  {

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
    static int32_t calibration_sum_current = 0;
    static int32_t calibration_sum_voltage = 0;
    int32_t sample;

    sample=sampleCurrent();
    sum_current += sample*sample;
    if(flag_control==false) { calibration_sum_current += sample;}

    sample=sampleVoltage();
    sum_voltage += sample*sample;
    if(flag_control==false) { calibration_sum_voltage += sample;}

    sample_count++;

    if(sample_count>=SAMPLES_PER_PERIOD)
    {
      if(flag_control==false)
      {
        calibration_current_sensor=calibration_sum_current/SAMPLES_PER_PERIOD;
        calibration_voltage_sensor=calibration_sum_voltage/SAMPLES_PER_PERIOD;  
        calibration_current_sensor=0;
        calibration_voltage_sensor=0;
      }
      else
      {
        calibration_current_sensor=0;
        calibration_voltage_sensor=0;
      }

      voltage_rms = sqrt(sum_voltage / SAMPLES_PER_PERIOD);
      current_rms = sqrt(sum_current / SAMPLES_PER_PERIOD);
      resistance_rms = voltage_rms / current_rms;
      temp_measured = (resistance_rms*T_slope-T_b);/* T=R*1/(TEMP_COEF*R_ZERO)+(1/TEMP_COEF-T_ZERO)*/

      if(voltage_rms>MAX_VOLTAGE_RMS)
      {
        errorHandler(ERROR_MAX_VOLTAGE_EXCEEDED);
      }
      else if(current_rms>MAX_CURRENT_RMS)
      {
        errorHandler(ERROR_MAX_CURRENT_EXCEEDED);
      }
      else if(current_rms>voltage_rms)
      {
        errorHandler(ERROR_CURRENT_HIGHER_THAN_VOLTAGE);
      }
      else if (resistance_rms > MAX_RESISTANCE)
      {
        errorHandler(ERROR_MAX_RESISTANCE_EXCEEDED);
      }
      else if (resistance_rms < r_zero)
      {
        errorHandler(ERROR_RESISTANCE_LOWER_THAN_RZERO);
      }
      else if (temp_measured == 0 && voltage_rms > 0.1*MAX_VOLTAGE_RMS && current_rms > 0.1*MAX_CURRENT_RMS)
      {
        errorHandler(ERROR_ZERO_TEMPERATURE);
      }
      else if(temp_measured>MAX_TEMPERATURE)
      {
        errorHandler(ERROR_MAX_TEMPERATURE_EXCEEDED);
      }

      sample_count = 0;
      sum_voltage = 0;
      sum_current = 0;
    }
    count_sampling = 0;
  }

  /*Control*/
  if (count_control >= COUNT_CONTROL && flag_control == true)
  {
    /*Check Pot value*/
    temp_pot=(analogRead(ANALOGpin_pot)*MAX_TEMPERATURE)>>ADC_RESOLUTION;
    if(abs(temp_pot-temp_sealing)>=POT_HYSTERESIS) /* Only change sealing temp if new temp is > hysteresis */
    {
      //temp_sealing=temp_pot;
    }

    controlTemp(temp_setpoint);
    count_control = 0;
  }
  else if (flag_control == false)
  {
    analogWrite(CTRLpin_PWM, MIN_DUTY_CYCLE);
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
