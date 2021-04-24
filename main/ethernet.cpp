
#include <SPI.h>
#include <NativeEthernet.h>
#include "ethernet.h"
#include "ethernetAPI.h"

// Enter a MAC address for your controller below.
static byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x40 };

// IP address in case DHCP fails
static IPAddress ip(192,168,2,2);

// Ethernet server
static EthernetServer server(80);

// Create aREST instance
static aREST rest = aREST();

// Custom functions accessible by the API
static void set_temp_seal(String setpoint) {
  temp_user_setpoint = setpoint.toInt();
}

static void set_temp_preheat(String preheat) {
  temp_preheat = preheat.toInt();
}

static int read_temp(){
  return temp_measured;
}

void InitEthernet(void)
{
  // Start Serial
  Serial.begin(BAUDRATE);
  Serial.print("\x1b[2J"); /*Clear screen*/

  // Function to be exposed
  rest.function("set_seal",set_temp_seal);
  rest.function("set_preheat",set_temp_preheat);
  rest.function("read_temp",read_temp);

  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("1");
  rest.set_name("Termorregulador Digital");

  // Start the Ethernet connection and the server
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    Ethernet.begin(mac, ip);
  }

  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
}

void ListenClient(){
  // listen for incoming clients
  EthernetClient client = server.available();
  rest.handle(client);
}




