
//#include <SPI.h>
#include <NativeEthernet.h>
#include "ethernet.h"
#include "ethernetAPI.h"
//#include <aREST.h>

// Enter a MAC address for your controller below.
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x40 };

// IP address in case DHCP fails
IPAddress ip(192,168,2,2);

// Ethernet server
EthernetServer server(80);

// Create aREST instance
aREST rest = aREST();


int read_temp(){
  return temp_measured;
}

void InitEthernet(void)
{
  rest.function("read_temp",read_temp);


  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("0001");
  rest.set_name("Termorregulador");

  // Start the Ethernet connection and the server
  if (Ethernet.begin(mac) == 0) {
    //Serial.println("Failed to configure Ethernet using DHCP");
    Ethernet.begin(mac, ip);
  }

  server.begin();
  //Serial.print("server is at ");
  //Serial.println(Ethernet.localIP());
}

void ListenClient(){
  // listen for incoming clients
  EthernetClient client = server.available();
  rest.handle(client);
  //Serial.println("\rclient\n");
}
