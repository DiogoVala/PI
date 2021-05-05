/*
  This a simple example of the aREST Library for Arduino (Uno/Mega/Due/Teensy)
  using the Ethernet library (for example to be used with the Ethernet shield).
  See the README file for more details.

  Written in 2014 by Marco Schwartz under a GPL license.
*/

// Libraries
#include <SPI.h>
#include <NativeEthernet.h>
#include <aREST.h>

#define pot 14  // Pot pin

// Enter a MAC address for your controller below.
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x40 };

// IP address in case DHCP fails
IPAddress ip(192,168,2,2);

// Ethernet server
EthernetServer server(80);

// Create aREST instance
aREST rest = aREST();

// Variables to be exposed to the API
int temperature;
int preheat;
int setpoint=0;

// Declare functions to be exposed to the API
void set_temp(String api_setpoint);

void setup(void)
{
  // Start Serial
  Serial.begin(115200);
  Serial.print("\x1b[2J"); /*Clear screen*/

  // Init variables and expose them to REST API
  temperature = 200;
  preheat = 150;
  rest.variable("curr_temperature",&temperature);
  rest.variable("preheat",&preheat);
  rest.variable("setpoint",&setpoint);

  // Function to be exposed
  rest.function("set_temp",set_temp);

  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("000");
  rest.set_name("Termorregulador");

  // Start the Ethernet connection and the server
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    Ethernet.begin(mac, ip);
  }

  //ADC resolution
  analogReadRes(12);

  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
}

void loop() {

  temperature = (analogRead(pot)*300)>>12;

  // listen for incoming clients
  EthernetClient client = server.available();
  rest.handle(client);
}

// Custom function accessible by the API
void set_temp(String api_setpoint) {
  setpoint = api_setpoint.toInt();

}
