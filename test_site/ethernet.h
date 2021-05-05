#ifndef ETHERNET_H
#define ETHERNET_H

extern volatile uint32_t temp_measured;

void InitEthernet(void);

void ListenClient();

#endif
