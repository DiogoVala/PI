#include "DisplayDriver.h"

void setup() {
  InitDisplay();
}
elapsedMillis timeElapsed;
void loop() {
  eventCheck();
  if(timeElapsed>100){
    updateGraph();
    timeElapsed=0;
  }
  
}
