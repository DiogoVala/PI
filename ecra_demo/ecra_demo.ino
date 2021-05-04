#include "DisplayDriver.h"

void setup() {
  InitDisplay();
}
elapsedMillis timeElapsed;
void loop() {
  eventCheck();
  if(timeElapsed>100){
    updateGraphs();
    timeElapsed=0;
  }
  
}
