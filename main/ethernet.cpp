/*
   File:   ethernet.cpp
   Author: Diogo Vala

   Overview: Ethernet management and API 
*/

#include <NativeEthernet.h>
#include "ethernet.h"
#include "ethernetAPI.h"
#include "config.h"
#include "memory.h"

volatile uint16_t network_port=80;
volatile uint8_t static_ip_arr[IP_ARRAY_SIZE] = {0};

byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x40 };

IPAddress static_ip;

EthernetServer server(0);

EthernetClient client;

aREST rest = aREST(); /* Create aREST instance */

/* Custom functions accessible by the API */
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
  flag_pot=false; /*Disallow potenciometer*/
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

int read_temp() {
  return temp_measured;
}

int read_voltage(){
  return (int)voltage_rms/100;
}

int read_current(){
  return (int)current_rms/100;
}

int print_log(){

  char intbuf[10]; /* Small buffer to store error code ( "nnn:err;" ) */
  memset(intbuf, 0, sizeof(intbuf));

  int16_t starting_idx=error_count-MAX_HTML_LOG_SIZE; /* Start from index that allows the most recent MAX_HTML_LOG_SIZE samples to be displayed */
  int16_t ending_idx;
  if(starting_idx<0) 
  {
    starting_idx=0;
  }
  if(error_count<MAX_HTML_LOG_SIZE)
  {
    ending_idx=error_count;
  }
  else
  {
    ending_idx=starting_idx+MAX_HTML_LOG_SIZE;
  }

  client.print("Error log: \n");
  for (int16_t i=starting_idx; i<ending_idx; i++)
  {
    snprintf(intbuf, sizeof(intbuf), "%d:%d; ", i+1, error_log[i]);
    client.print(intbuf); /* Print error codes to local server */
  }
}

int clear_log(){
  error_count=0;
  writeInt16ToEEPROM(ADDR_ERROR_COUNT, error_count);
}


void InitEthernet()
{
  server = EthernetServer(network_port);

  /* Funcions available on the API */
  rest.function("set_seal",set_temp_seal);
  rest.function("set_preheat",set_temp_preheat);
  rest.function("read_temp",read_temp);
  rest.function("read_voltage",read_voltage);
  rest.function("read_current",read_current);
  rest.function("log",print_log);
  rest.function("clrlog",clear_log);

  /* Device identification */
  rest.set_id("0001");
  rest.set_name("Termorregulador");

  /* Start server using static IP */
  Ethernet.begin(mac, static_ip); 
}

int8_t linkStatus(){
  if(Ethernet.linkStatus() == LinkON){
    return ETHERNET_ONLINE;
  }
  else {
    return ETHERNET_OFFLINE;
  }
}

void getIP(){
  Serial1.print(Ethernet.localIP()); /* Print directly to display - Not the best solution. */
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
  server = EthernetServer(0); /* Only solution to stopping the server. Doesn't allow new connection.*/
}

void ListenClient(){
  client = server.available();
  rest.handle(client);
}
