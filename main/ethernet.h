#ifndef ETHERNET_H
#define ETHERNET_H

#define IP_ARRAY_SIZE 4
#define IP_LENGTH 15

#define ETHERNET_ONLINE 0
#define ETHERNET_OFFLINE -1
#define ERROR_INVALID_IP -2
#define ERROR_INVALID_NETWORK_PORT -3

extern volatile uint32_t temp_sealing;
extern volatile uint32_t temp_preheat;
extern volatile uint32_t temp_measured;
extern volatile float current_rms;
extern volatile float voltage_rms;



/********************************************************************
   Function:    InitEthernet()
   Precondition: 
   Input:      
   Returns:

   Side Effects:

   Overview:    Initializes ethernet module

   Note:     Uses port 0 as default port

 ********************************************************************/
int8_t InitEthernet();


/********************************************************************
   Function:    linkStatus()
   Precondition: Ethernet module must be initialized
   Input:      
   Returns:		ETHERNET_ONLINE (0) / ETHERNET_OFFLINE (-1)

   Side Effects:

   Overview:    checks ethernet status

   Note:     

 ********************************************************************/
int8_t linkStatus();

int8_t setNetPort(uint32_t port);

int8_t setIP(const char* ip);


/********************************************************************
   Function:    getIP()
   Precondition: Ethernet module must be initialized
   Input:      
   Returns:		string with IP

   Side Effects:

   Overview:    gets current local IP

   Note:     

 ********************************************************************/
const char* getIP();


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
