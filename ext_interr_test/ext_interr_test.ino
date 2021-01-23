#define HWSERIAL Serial1
#define ledPin LED_BUILTIN
#define digpin 2

static int16_t baudrate = 9600;
volatile int adcval;

void setup() {
  // put your setup code here, to run once:
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
  // put your main code here, to run repeatedly:
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
  
  
  
  // FOR USB CONNECTION
  if (Serial.available() > 0) {
    incomingByte = Serial.read(); // Byte is received in DECIMAL format
    Serial.print("USB received ASCII: ");
    Serial.println(incomingByte, DEC);
  }
}
