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
  //Attach interrupt to pin 2
  attachInterrupt(digitalPinToInterrupt(digpin), extISR, RISING);
  pinMode(digpin, OUTPUT);
}
volatile bool flag=false;

void extISR() {
  flag=true;
}

elapsedMillis timer; // Counts microseconds since program start
volatile int i=0;

void loop() {

  if(flag==true){
    Serial.print("INTERRUPT "); //Clear screen
    Serial.println(i++);
    flag=false;
  }

  // Set pin 3 to high/low every 1 second
  if (timer<1000){
    digitalWrite(digpin, HIGH);
  }
  else if (timer<2000){
    digitalWrite(digpin, LOW);
  }
  else{
    timer=0;
  }
}
