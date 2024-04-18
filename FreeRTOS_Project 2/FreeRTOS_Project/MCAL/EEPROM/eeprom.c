/*
 * eeprom.c
 *
 *  Created on: Apr 13, 2024
 *      Author: seif alrahman
 */
#include "tm4c123gh6pm_registers.h"
#include "eeprom.h"

uint32 EEPROMInit(void){

    SYSCTL_RCGCEEPROM_REG|=(1<<0) ;
    while(!(SYSCTL_PREEPROM_REG&(1<<0))) ;
    while( EEPROM_EEDONE_REG & (1<<0) )   ;/*Poll the WORKING bit in the EEPROM Done Status (EEDONE) register until it is clear, indicating
                                            that the EEPROM has completed its power-on initialization. When WORKING=0, continue.*/
    if( (EEPROM_EESUPP_REG & (1<<2)) || (EEPROM_EESUPP_REG & (1<<3)) ){
        /* #error "Either PRETRY or ERETRY bits is set " Read the PRETRY and ERETRY bits in the EEPROM Support Control and Status (EESUPP)
                                                    register. If either of the bits are set, return an error, else continue.*/
    }else {

        SYSCTL_SREEPROM_REG |=(1<<0) ; /* Reset the EEPROM module using the EEPROM Software Reset (SREEPROM) register at
                                          offset 0x558 in the System Control register space*/

        while(!(SYSCTL_PREEPROM_REG&(1<<0))) ;

        while(EEPROM_EEDONE_REG & (1<<0) ); /* Poll the WORKING bit in the EEPROM Done Status (EEDONE) register to determine when it is
                                                clear. When WORKING=0, continue.*/

        if( (EEPROM_EESUPP_REG & (1<<2)) || (EEPROM_EESUPP_REG & (1<<3)) ){
            /* #error "Either PRETRY or ERETRY bits is set " Read the PRETRY and ERETRY bits in the EEPROM Support Control and Status (EESUPP)
                                                            register. If either of the bits are set, return an error, else continue.*/
            }else {
                return 1;
            }

    }
    return 0 ;
}



void EEPROMWrite(uint32 data,uint8 addr,uint8 blk)
{
    EEPROM_EEBLOCK_REG = blk;
    EEPROM_EEOFFSET_REG =  addr;
    EEPROM_EERDWRINC_REG = data;
    while(EEPROM_EEDONE_REG);
    /*If EEDONE==0, then the write completed successfully. If EEDONE!=0, then an error occurred
      and the source of the error is given by the set bit(s).*/
}

uint32 EEPROMRead(uint8 addr,uint8 blk)
{
    uint32 data;
    EEPROM_EEBLOCK_REG = blk;
    EEPROM_EEOFFSET_REG =  addr;
    data = EEPROM_EERDWRINC_REG;
    while(EEPROM_EEDONE_REG );
    return data;
}

