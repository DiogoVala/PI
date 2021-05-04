#include "DisplayDriver.h"

void setup() {
  InitDisplay();
}

void loop() {
  eventCheck();
  updateGraph();
}
