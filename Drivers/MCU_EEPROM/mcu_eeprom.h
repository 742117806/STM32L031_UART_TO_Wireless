
#ifndef __MCU_EEPROM_H
#define __MCU_EEPROM_H
#include "stm32l0xx.h"




void EEPROM_ReadBytes(uint16_t Addr,uint8_t *Buffer,uint16_t Length);
void EEPROM_ReadWords(uint16_t Addr,uint16_t *Buffer,uint16_t Length);
void EEPROM_WriteBytes(uint16_t Addr,uint8_t *Buffer,uint16_t Length);
void EEPROM_WriteWords(uint16_t Addr,uint16_t *Buffer,uint16_t Length);
void MCU_EEPRomTest(void);

#endif

