#include "EEPROM_utils.h"
#include <stdint.h>
#include <EEPROM.h>

void writeInt8ToEEPROM(uint16_t addr, uint8_t value){
	EEPROM.write(addr, value);
}

void writeInt16ToEEPROM(uint16_t addr, uint16_t value){
	EEPROM.write(addr, 	 value & 0xFF);
	EEPROM.write(addr+1, value >> 8);
}

void writeInt32ToEEPROM(uint16_t addr, uint32_t value){
	EEPROM.write(addr,   value & 0xFF);
	EEPROM.write(addr+1, value >> 8);
	EEPROM.write(addr+2, value >> 16);
	EEPROM.write(addr+3, value >> 24);
}

uint8_t readInt8FromEEPROM(uint16_t addr){
	return EEPROM.read(addr);
}

uint16_t readInt16FromEEPROM(uint16_t addr){
	return ((EEPROM.read(addr+1) << 8) + EEPROM.read(addr));
}

uint32_t readInt32FromEEPROM(uint16_t addr){
	return ((EEPROM.read(addr+3) << 24) + (EEPROM.read(addr+2) << 16) + (EEPROM.read(addr+1) << 8) + (EEPROM.read(addr)));
}

