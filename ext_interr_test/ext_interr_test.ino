/* Tests external interrupt and prints via USB
 *  Diogo Vala
 */

#define HWSERIAL Serial1
#define digpin 2

static int16_t baudrate = 9600;
/*  Data bits - 8
    Parity    - None
    Stop bits - 1
*/
volatile int adcval;

void setup() {
  Serial.begin(baudrate); //
  attachInterrupt(digitalPinToInterrupt(digpin), extISR, RISING);
  pinMode(3, OUTPUT);
}
volatile bool flag=false;

void extISR() {
  flag=true;
}

elapsedMillis timer; // Counts microseconds since program start

void loop() {
  int incomingByte;

  if(flag==true){
    Serial.println("TEST"); //Clear screen
    flag=false;
  }

  if (timer<1000){
    digitalWrite(3, HIGH);
  }
  else if (timer<2000){
    digitalWrite(3, LOW);
  }
  else{
    timer=0;
  }
}
