/*
 * eeprom.h
 *
 *  Created on: Apr 13, 2024
 *      Author: seif alrahman
 */

#ifndef MCAL_EEPROM_EEPROM_H_
#define MCAL_EEPROM_EEPROM_H_

#include "std_types.h"

#define EEPROM_EEDONE_REG    (*((volatile  uint32 *)0x400AF018 ))
#define EEPROM_EESUPP_REG    (*((volatile  uint32 *)0x400AF01C ))
#define EEPROM_EEBLOCK_REG   (*((volatile  uint32 *)0x400AF004 ))
#define EEPROM_EEOFFSET_REG  (*((volatile  uint32 *)0x400AF008 ))
#define EEPROM_EERDWRINC_REG (*((volatile  uint32 *)0x400AF014 ))
uint32 EEPROMInit(void);
void EEPROMWrite(uint32 data,uint8 addr,uint8 blk);
uint32 EEPROMRead(uint8 addr,uint8 blk);

#endif /* MCAL_EEPROM_EEPROM_H_ */
