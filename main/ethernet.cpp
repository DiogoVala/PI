#include <NativeEthernet.h>
#include "ethernet.h"
#include "ethernetAPI.h"

#define ETHERNET_ONLINE 0
#define ETHERNET_OFFLINE -1

volatile uint16_t network_port=80;

// Enter a MAC address for your controller below.
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x40 };

// IP address in case DHCP fails
IPAddress ip(192,168,2,2);

// Ethernet server
EthernetServer server(0);

// Create aREST instance
aREST rest = aREST();

// Custom functions accessible by the API
void set_temp_seal(String setpoint) {
  temp_sealing = setpoint.toInt();
}

void set_temp_preheat(String preheat) {
  temp_preheat = preheat.toInt();
}

int read_temp(){
  return temp_measured;
}

float read_voltage(){
  return voltage_rms;
}

float read_current(){
  return current_rms;
}

void InitEthernet()
{
  server = EthernetServer(network_port);

  // Function to be exposed
  rest.function("set_seal",set_temp_seal);
  rest.function("set_preheat",set_temp_preheat);
  rest.function("read_temp",read_temp);
  rest.function("read_voltage",read_voltage);
  rest.function("read_current",read_current);

  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("0001");
  rest.set_name("Termorregulador");

  // Start the Ethernet connection and the server
  if (Ethernet.begin(mac) == 0) {
    //Serial.println("Failed to configure Ethernet using DHCP");
    Ethernet.begin(mac, ip);
  }

  server.begin();

}

int8_t linkStatus(){
  if(Ethernet.linkStatus() == LinkON){
    return ETHERNET_ONLINE;
  }
  else {
    return ETHERNET_OFFLINE;
  }
}

const char* getIP(){
  Serial1.print(Ethernet.localIP());
}

void stopEthernet(){
  server = EthernetServer(0);
}

void ListenClient(){
  EthernetClient client = server.available();
  rest.handle(client);
}
