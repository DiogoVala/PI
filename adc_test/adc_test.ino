#define HWSERIAL Serial1
#define ledPin LED_BUILTIN
#define A0Pin 14
#include "inttypes.h"

IntervalTimer myTimer;
static float timerPeriod = 0.1; // x seconds
volatile int conversionTime;
volatile int adcval;


static int16_t baudrate = 9600;
/*  Data bits - 8
    Parity    - None
    Stop bits - 1
*/

void setup() {
  Serial.begin(baudrate); //
  analogReadRes(12); //12 bit ADC
  myTimer.begin(myPrint, timerPeriod*1000000);  // Calls blink
}

void myPrint(){
  Serial.print("\r\n");
  Serial.print("\n\x1b[2J\r"); //Clear screen
  Serial.print("\rConversion Time: ");
  Serial.println(conversionTime);
  Serial.print("\rADC Value: ");
  Serial.println(adcval);
}

elapsedMicros timer; // Counts microseconds since program start

void loop() {
  int incomingByte;
  
  timer = 0;
  adcval = analogRead(A0Pin);
  conversionTime = timer; // Conversion takes 17-18us for 10bit and 21us for 12bit

  // FOR USB CONNECTION
  if (Serial.available() > 0) {
    incomingByte = Serial.read(); // Byte is received in DECIMAL format
    Serial.print("USB received ASCII: ");
    Serial.println(incomingByte, DEC);
  }
}
