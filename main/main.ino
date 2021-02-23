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


static int16_t baudrate = 9600;
volatile int8_t state = 0;
/*  0 - Sistema Desligado
 *  1 - Sistema Ligado
 *  2 - Inicio de ciclo
 *  3 - Pre-heat
 *  4 - Aumentar Temp
 *  5 - Selagem
 *  6 - Alarme
*/
volatile int16_t setpoint=0; // 0 to ~400º
volatile int16_t temp_preheat=0;  // 0 to ~400º
volatile int16_t temp=0; // 0 to ~400º
volatile int16_t period=0; //0 to ~21000
volatile int16_t duty=0; // 0 to 4095


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
    state = 6;
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
    state = 6;
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
    state = 6;
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
  pinMode(A0Pin, INPUT); 
  pinMode(A1Pin, INPUT);
  pinMode(A2Pin, INPUT);
  pinMode(D0Pin, INPUT);
  pinMode(D1Pin, INPUT);
  pinMode(D2Pin, INPUT);
  pinMode(D3Pin, INPUT);
  pinMode(D4Pin, INPUT);
  pinMode(D5Pin, OUTPUT);
  pinMode(D6Pin, OUTPUT);
  pinMode(D7Pin, OUTPUT);
  pinMode(D8Pin, INPUT);
  digitalWrite(D5Pin, LOW);
  digitalWrite(D6Pin, LOW);
  digitalWrite(D7Pin, LOW);
  analogWriteResolution(12); // 2-15bits 
  analogWrite(D6Pin, duty); //PWM Output

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
}

void alarme()
{
}

void loop() {
  // put your main code here, to run repeatedly:
  
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
  duty = analogRead(A0Pin);

  Serial.println(duty);

}
