#ifndef ETHERNET_H
#define ETHERNET_H

extern volatile uint16_t temp_user_setpoint;
extern volatile uint16_t temp_preheat;
extern volatile uint16_t temp_measured;

void InitEthernet(void);

void ListenClient();

#endif