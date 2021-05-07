#ifndef ETHERNET_H
#define ETHERNET_H

extern volatile uint32_t temp_sealing;
extern volatile uint32_t temp_preheat;
extern volatile uint32_t temp_measured;
extern volatile float current_rms;
extern volatile float voltage_rms;

void InitEthernet(void);

void ListenClient();

#endif
