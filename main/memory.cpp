/*
   File:   memory.cpp
   Author: Diogo Vala

   Overview: EEPROM memory management
*/

#include <EEPROM.h>
#include <stdint.h>
#include "memory.h"

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

void loadMemory(){

  temp_preheat = readInt32FromEEPROM(ADDR_TEMP_PREHEAT);
  temp_sealing = readInt32FromEEPROM(ADDR_TEMP_SEALING);
  pid_kp = (float)readInt32FromEEPROM(ADDR_PID_KP)/NUMBOX_PID_K;
  pid_ki =(float)readInt32FromEEPROM(ADDR_PID_KI)/NUMBOX_PID_K;
  pid_kd = (float)readInt32FromEEPROM(ADDR_PID_KD)/NUMBOX_PID_K;
  pid_int_limit = readInt32FromEEPROM(ADDR_PID_INT_LIM);
  temp_coef = (float)readInt32FromEEPROM(ADDR_TEMP_COEF)/NUMBOX_TEMP_COEF;
  r_zero = (float)readInt16FromEEPROM(ADDR_R_ZERO)/NUMBOX_R_ZERO;
  network_port = readInt16FromEEPROM(ADDR_NETWORK_PORT);
  error_count = readInt16FromEEPROM(ADDR_ERROR_COUNT);

  if(error_count>(ERROR_LOG_SIZE-1))
  {
    error_count=ERROR_LOG_SIZE-1;
  }
  for (uint16_t i = 0; i<error_count; i++)
  {
    error_log[i]=readInt8FromEEPROM(ADDR_ERROR_LOG+i);
  }

  for (uint8_t i = 0; i<IP_ARRAY_SIZE; i++)
  {
    static_ip_arr[i] = readInt8FromEEPROM(ADDR_STATIC_IP+i);
  }
}
