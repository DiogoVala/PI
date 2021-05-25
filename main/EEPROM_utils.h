#ifndef EEPROM_ADDRESSES_H
#define EEPROM_ADDRESSES_H

#include <stdint.h>

/*
	EEPROM Memory Map
	0-temp_preheat
	1-temp_preheat
	2-temp_preheat
	3-temp_preheat
	4-temp_sealing
	5-temp_sealing
	6-temp_sealing
	7-temp_sealing
	8-TEMP_COEF
	9-TEMP_COEF
	10-TEMP_COEF
 	11-TEMP_COEF
	12-R_ZERO
	13-R_ZERO
	14-PID_KP
	15-PID_KP
	16-PID_KP
	17-PID_KP
	18-PID_KI
	19-PID_KI
	20-PID_KI
	21-PID_KI
	22-PID_KD
	23-PID_KD
	24-PID_KD
	25-PID_KD
	26-PID_INT_LIM
	27-PID_INT_LIM
	28-PID_INT_LIM
	29-PID_INT_LIM
	30-NETWORK_PORT
	31-NETWORK_PORT
	32-IP
	33-IP
	34-IP
	35-IP
	36-Error count
	37-Error count
	38-Error Log
	...
	86-Error Log
	87-
*/
#define ADDR_TEMP_PREHEAT 0
#define ADDR_TEMP_SEALING 4
#define ADDR_TEMP_COEF 8
#define ADDR_R_ZERO 12
#define ADDR_PID_KP 14
#define ADDR_PID_KI 18
#define ADDR_PID_KD 22
#define ADDR_PID_INT_LIM 26
#define ADDR_NETWORK_PORT 30
#define ADDR_STATIC_IP 32
#define ADDR_ERROR_COUNT 36
#define ADDR_ERROR_LOG 38


void writeInt8ToEEPROM(uint16_t addr, uint8_t value);


void writeInt16ToEEPROM(uint16_t addr, uint16_t value);


void writeInt32ToEEPROM(uint16_t addr, uint32_t value);


uint8_t readInt8FromEEPROM(uint16_t addr);


uint16_t readInt16FromEEPROM(uint16_t addr);


uint32_t readInt32FromEEPROM(uint16_t addr);


#endif