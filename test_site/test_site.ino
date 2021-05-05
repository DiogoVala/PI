
#include "ethernet.h"
#define ANALOGpin_current 15

volatile uint32_t  temp_measured = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  analogReadRes(12);
  pinMode(ANALOGpin_current, INPUT);

  InitEthernet();
  
}

elapsedMillis time;
void loop() {

  ListenClient();
  if (time>100){
    temp_measured = analogRead(ANALOGpin_current)*300/4095;
    Serial.println(temp_measured);
  }
  // put your main code here, to run repeatedly:

}
