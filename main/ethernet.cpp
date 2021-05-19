#include <NativeEthernet.h>
#include "config.h"
#include "ethernet.h"
#include "ethernetAPI.h"
#include <EEPROM.h>
#include "EEPROM_addresses.h"

volatile uint16_t network_port;

// Enter a MAC address for your controller below.
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x40 };

volatile uint8_t static_ip_arr[IP_ARRAY_SIZE] = {0};

// IP address in case DHCP fails
IPAddress static_ip;

// Ethernet server
EthernetServer server(0);

// Create aREST instance
aREST rest = aREST();

// Custom functions accessible by the API
void set_temp_seal(String setpoint) {
  uint32_t local_sealing = setpoint.toInt();
  if(local_sealing<0)
  {
    temp_sealing = 0;
  }
  else if(local_sealing>MAX_SEALING)
  {
    temp_sealing = MAX_SEALING;
  }
  else
  {
    temp_sealing=local_sealing;
  }
}

void set_temp_preheat(String preheat) {
  uint32_t local_preheat = preheat.toInt();
  if(local_preheat<0)
  {
    temp_preheat = 0;
  }
  else if(local_preheat>MAX_PREHEAT)
  {
    temp_preheat = MAX_PREHEAT;
  }
  else
  {
    temp_preheat=local_preheat;
  }
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

int8_t InitEthernet()
{
  EEPROM.get(ADDRESS_NETWORK_PORT, network_port);

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

  Ethernet.begin(mac, static_ip); /* Connect using static IP */

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

int8_t setNetPort(uint32_t port) {
  int8_t error_code=0;

  Serial.println(port);
  if(port > INT16_MAX)
  {
    error_code = ERROR_INVALID_NETWORK_PORT;
  }
  else
  {
    network_port=port;
    EEPROM.put(ADDRESS_NETWORK_PORT, port);
  }
  return error_code;
}

void strToIP(const char * str) {

  uint8_t j=0;
  static_ip_arr[j]=0;

  for(int i=0; i<IP_LENGTH; i++)
  {
    if(str[i]!='.' && str[i]!='\0')
    {
      static_ip_arr[j]=static_ip_arr[j]*10+(int)str[i]-48; /* Save valid digit as integer */
    }
    else
    {
      j++;
      static_ip_arr[j]=0;
    }
  }
}

int8_t setIP(const char* ip) {
  int8_t error_code=0;

  if(static_ip.fromString(ip) ==  0)
  {
    error_code = ERROR_INVALID_IP;
  }
  else
  {
    strToIP(ip);
    for(uint8_t i=0; i<IP_ARRAY_SIZE; i++)
    {
      EEPROM.put(ADDRESS_STATIC_IP+i, static_ip_arr[i]);
    }
  }
  return error_code;
}

void stopEthernet(){
  server = EthernetServer(0);
}

void ListenClient(){
  EthernetClient client = server.available();
  rest.handle(client);
}
