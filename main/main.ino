#define A0Pin 14 // Analog input 0 - Pot
#define A1Pin 15 // Analog input 1 - Current
#define A2Pin 16 // Analog input 2 - Voltage

#define D0Pin 2 // Digital 0 - Enable
#define D1Pin 3 // Digital 1 - Start 
#define D2Pin 4 // Digital 2 - Pre-heat
#define D3Pin 5 // Digital 3 - Sealing
#define D4Pin 6 // Digital 4 - Reset 
#define D5Pin 7 // Digital 5 - Alarm

#define D6Pin 8 // Digital 6 - PWM output for controller
#define D7Pin 9 // Digital 7 - Controller On/Off
#define D8Pin 10 // Digital 8 - Passagem por zero

//Uart
static uint16_t baudrate = 9600;
//Sensores
static double current_K1 = 29.4643; // Constante de conversão de corrente em tensão do circuito de condicionamento
static uint16_t current_K2 = 1000; // Constante de conversão do transformador de corrente
static double voltage_K1 = 41.14; // Constante do divisor resistivo do circuito de condicionamento
//Controlo
static uint16_t PID_Kp = 1;
static uint16_t PID_Ki = 1;
static uint16_t PID_Kd = 1;
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

void ENABLE() {
  if(digitalRead(D0Pin) == 0)
  {
    state = 0; // Sistema desligado
  }
  else
  {
    state = 1; // Sistema ligado
  }
}

void START() {
  if(digitalRead(D1Pin) == 0 && state == 2)
  {
    state = 1; // Sistema Ligado
  }
  else if( digitalRead(D1Pin) == 1 && state == 1)
  {
    state = 2; // Inicio de ciclo
  }
  else
  {
    state = 6; // Alarme
  }
}

void PREHEAT() {
  if(digitalRead(D2Pin) == 0 && state == 2)
  {
    state = 2; // Inicio de ciclo
  }
  else if( digitalRead(D2Pin) == 1 && state == 2)
  {
    state = 3; // Pre-heat
  }
  else
  {
    state = 6; // Alarme
  }
}

void SEALING() {
  if(digitalRead(D3Pin) == 0 && state == 2)
  {
    state = 2; // Inicio de ciclo
  }
  else if( digitalRead(D3Pin) == 1 && ( state == 2 || state ==3))
  {
    state = 4; // Aumentar Temp
  }
  else if( digitalRead(D3Pin) == 0 && state == 5)
  {
    state = 2; // Inicio de ciclo
  }
  else
  {
    state = 6; // Alarme
  }
}

void RESET() {
  if(digitalRead(D4Pin == 1))
  {
    state = 0;  
  }
}

elapsedMicros timer;
volatile bool periodflag=false;
void ZEROPASS() {
  if(digitalRead(D3Pin) == 1 && periodflag == false)
  {
    period=timer;
    timer = 0;
    periodflag=true;
    
  }
  else if( digitalRead(D3Pin) == 1 && periodflag == true)
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
  attachInterrupt(digitalPinToInterrupt(D0Pin), ENABLE, CHANGE); // Enable
  attachInterrupt(digitalPinToInterrupt(D1Pin), START, CHANGE); // Start 
  attachInterrupt(digitalPinToInterrupt(D2Pin), PREHEAT, CHANGE); // Pre-heat
  attachInterrupt(digitalPinToInterrupt(D3Pin), SEALING, CHANGE); // Sealing
  attachInterrupt(digitalPinToInterrupt(D4Pin), RESET, RISING); // Reset
  attachInterrupt(digitalPinToInterrupt(D8Pin), ZEROPASS, RISING); // Passagem por zero
  pinMode(A0Pin, INPUT); // Pot
  pinMode(A1Pin, INPUT); // Current sensor
  pinMode(A2Pin, INPUT); // Voltage sensor
  pinMode(D0Pin, INPUT); // Enable
  pinMode(D1Pin, INPUT); // Start 
  pinMode(D2Pin, INPUT); // Pre-heat
  pinMode(D3Pin, INPUT); // Sealing
  pinMode(D4Pin, INPUT); // Reset 
  pinMode(D5Pin, OUTPUT); // Alarm
  pinMode(D6Pin, OUTPUT); // PWM output for controller
  pinMode(D7Pin, OUTPUT); // Controller On/Off
  pinMode(D8Pin, INPUT); // Passagem por zero
  digitalWrite(D5Pin, LOW);
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
