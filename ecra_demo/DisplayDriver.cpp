#include "DisplayDriver.h"

NexButton b0 = NexButton(1, 1, "b0");  // Button added
NexButton b1 = NexButton(2, 8, "b1");  // Button added
NexNumber n0 = NexNumber(2, 2, "n0"); // Text box added, so we can read it
NexNumber n1 = NexNumber(2, 6, "n1"); // Text box added, so we can read it
NexWaveform s0 = NexWaveform(3, 1, "s0");
NexPage page1 = NexPage(1, 0, "page1");  // Page added as a touch event
NexPage page3 = NexPage(3, 0, "page3");  // Page added as a touch event
NexTouch *nex_listen_list[] = 
{
  &b0,
  &b1,  // Button added
  &page1,
  &page3,
  NULL  // String terminated
};  // End of touch event list

volatile uint32_t preheat = 150;
volatile uint32_t sealing = 250;

volatile uint32_t current_page = 0;

void b0PushCallback(void *ptr)
{
  digitalWrite(13, HIGH);  // Turn ON internal LED
}

void b0PopCallback(void *ptr)
{
  digitalWrite(13, LOW);  // Turn ON internal LED

  Serial1.print("n0.val=");
  Serial1.print(preheat);
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("n1.val=");
  Serial1.print(sealing);
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);

}

void b1PushCallback(void *ptr)
{
  uint32_t local_preheat=0;
  uint32_t local_sealing=0;
  digitalWrite(13, HIGH);  // Turn ON internal LED
  n0.getValue(&local_preheat);
  n1.getValue(&local_sealing);

  preheat=local_preheat;
  sealing=local_sealing;

}  // End of press event

void b1PopCallback(void *ptr)
{
  digitalWrite(13, LOW);  // Turn OFF internal LED
}  // End of release event

void page1PushCallback(void *ptr)
{
  current_page = 1;  // Set variable as 0 so from now on arduino knows page 0 is loaded on the display
  Serial.println(current_page);
}  // End of press event

void page3PushCallback(void *ptr)
{
  current_page = 3;  // Set variable as 0 so from now on arduino knows page 0 is loaded on the display
  Serial.println(current_page);
}  // End of press event


void InitDisplay() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  pinMode(14, INPUT);
  analogReadRes(12);

  Serial1.begin(115200);  // Display Baudrate
  Serial.begin(9600); // Terminal baudrate

  b0.attachPop(b0PopCallback);
  b1.attachPush(b1PushCallback);  // Button press
  b1.attachPop(b1PopCallback);  // Button release
  page1.attachPush(page1PushCallback);  // Page press event
  page3.attachPush(page3PushCallback);  // Page press event

}

void updateGraph(){
  uint32_t pot = 0;
  if(current_page==3)
    pot=analogRead(14);
    pot=pot*255/4095;
    s0.addValue(0,pot);
}

void eventCheck() {
  nexLoop(nex_listen_list);  // Check for any touch event
}
