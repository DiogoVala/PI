#include <NativeEthernet.h>
#include "config.h"
#include "ethernet.h"
#include "ethernetAPI.h"
#include <EEPROM.h>
#include "EEPROM_utils.h"

volatile uint16_t network_port=80;

// Enter a MAC address for your controller below.
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x40 };

volatile uint8_t static_ip_arr[IP_ARRAY_SIZE] = {0};

// IP address in case DHCP fails
IPAddress static_ip;

// Ethernet server
EthernetServer server(0);

//Ethernet client
EthernetClient client;

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

int read_voltage(){
  return (int)voltage_rms/100;
}

int read_current(){
  return (int)current_rms/100;
}

int print_log(){
  char buffer[10000];
  char intbuf[10];
  memset(buffer, 0, sizeof(buffer)); /* Clear buffer for IP */
  memset(intbuf, 0, sizeof(intbuf)); /* Clear buffer for IP */

  for(uint16_t i = 0; i<error_count; i++)
  {
    snprintf(intbuf, sizeof(intbuf), "%d:%d; ", i+1, error_log[i]);
    strcat(buffer, intbuf);
  } 
  client.print("Error log: \n");
  client.println(buffer);
}


int8_t InitEthernet()
{
  server = EthernetServer(network_port);

  // Function to be exposed
  rest.function("set_seal",set_temp_seal);
  rest.function("set_preheat",set_temp_preheat);
  rest.function("read_temp",read_temp);
  rest.function("read_voltage",read_voltage);
  rest.function("read_current",read_current);
  rest.function("log",print_log);

  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("0001");
  rest.set_name("Termorregulador");

  Serial.println(static_ip);
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

  if(port > UINT16_MAX)
  {
    error_code = ERROR_INVALID_NETWORK_PORT;
  }
  else
  {
    network_port=port;
    writeInt16ToEEPROM(ADDR_NETWORK_PORT, (uint16_t)network_port);
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

  strToIP(ip);
  for(uint8_t i=0; i<IP_ARRAY_SIZE; i++)
  {
    writeInt8ToEEPROM(ADDR_STATIC_IP+i, static_ip_arr[i]);
    if(static_ip_arr[i]>255)
    {
      error_code = ERROR_INVALID_IP;
      break;
    }
    static_ip[i]=static_ip_arr[i];
  }
  return error_code;
}

void stopEthernet(){
  server = EthernetServer(0);
}

void ListenClient(){
  client = server.available();
  rest.handle(client);
}
