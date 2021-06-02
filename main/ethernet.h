/*
   File:   ethernet.h
   Author: Diogo Vala

   Overview: Ethernet management and API 
*/

#ifndef ETHERNET_H
#define ETHERNET_H

#include "errors.h"

#define MAX_HTML_LOG_SIZE 200 /* Maximum number of error codes to show */
#define IP_ARRAY_SIZE 4 /* 111.222.333.444 - 4 numbers to store */
#define IP_LENGTH 16 /* Max length of IP string*/

/* Error codes */
#define ETHERNET_ONLINE 0
#define ETHERNET_OFFLINE -1
#define ERROR_INVALID_IP -2
#define ERROR_INVALID_NETWORK_PORT -3

/* Variables available on the API */
extern volatile uint32_t temp_sealing;
extern volatile uint32_t temp_preheat;
extern volatile uint32_t temp_measured;
extern volatile float current_rms;
extern volatile float voltage_rms;
extern volatile uint16_t error_count;
extern volatile uint8_t error_log[ERROR_LOG_SIZE];
extern volatile bool flag_pot;


/********************************************************************
   Function:    InitEthernet()
   Precondition: 
   Input:      
   Returns:

   Side Effects:

   Overview:    Initializes ethernet module

   Note:     Uses port 0 as default port

 ********************************************************************/
void InitEthernet();


/********************************************************************
   Function:    linkStatus()
   Precondition: Ethernet module must be initialized
   Input:      
   Returns:		ETHERNET_ONLINE (0) / ETHERNET_OFFLINE (-1)

   Side Effects:

   Overview:    Checks ethernet status

   Note:     

 ********************************************************************/
int8_t linkStatus();


/********************************************************************
   Function:    setNetPort()
   Precondition: 
   Input:      network port (0 - 65535)
   Returns:    ERROR_INVALID_NETWORK_PORT / 0

   Side Effects:

   Overview:    Set network port

   Note:     

 ********************************************************************/
int8_t setNetPort(uint32_t port);


/********************************************************************
   Function:    setIP()
   Precondition: 
   Input:      ip string in the format "nnn.nnn.nnn.nnn"
               where nnn is (0-255)
   Returns:    ERROR_INVALID_IP / 0

   Side Effects:

   Overview:    Set network port

   Note:     

 ********************************************************************/
int8_t setIP(const char* ip);


/********************************************************************
   Function:    getIP()
   Precondition: Ethernet module must be initialized
   Input:      
   Returns:		

   Side Effects:

   Overview:    prints IP string to display

   Note:     Not the best solution

 ********************************************************************/
void getIP();


/********************************************************************
   Function:    stopEthernet()
   Precondition: Ethernet module must be initialized
   Input:      
   Returns:

   Side Effects:

   Overview:    Sets port to 0

   Note:     Server cannot be stopped, but port can be changed to one
   that doesn't allow connections.

 ********************************************************************/
void stopEthernet();


/********************************************************************
   Function:    ListenClient()
   Precondition: Ethernet module must be initialized
   Input:      
   Returns:

   Side Effects:

   Overview:    Listens for client connections

   Note:     

 ********************************************************************/
void ListenClient();

#endif
