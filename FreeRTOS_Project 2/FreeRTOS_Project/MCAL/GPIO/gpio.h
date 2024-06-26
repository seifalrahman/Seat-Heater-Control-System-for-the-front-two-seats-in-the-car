
#ifndef GPIO_H_
#define GPIO_H_

#include "std_types.h"

#define GPIO_PORTF_PRIORITY_MASK      0xFF1FFFFF
#define GPIO_PORTF_PRIORITY_BITS_POS  21
#define GPIO_PORTF_INTERRUPT_PRIORITY 5

#define PRESSED                ((uint8)0x00)
#define RELEASED               ((uint8)0x01)

void GPIO_BuiltinButtonsLedsInit(void);
void GPIO_ExtraExternalButtonAndLedsInit (void) ;


void ExternalGPIO_RedLedOn(void);
void ExternalGPIO_BlueLedOn(void);
void ExternalGPIO_GreenLedOn(void);
void GPIO_RedLedOn(void);
void GPIO_BlueLedOn(void);
void GPIO_GreenLedOn(void);
void ExternalGPIO_GreenLedOff(void);
void ExternalGPIO_BlueLedOff(void);
void ExternalGPIO_RedLedOff(void);
void GPIO_RedLedOff(void);
void GPIO_BlueLedOff(void);
void GPIO_GreenLedOff(void);

void GPIO_RedLedToggle(void);
void GPIO_BlueLedToggle(void);
void GPIO_GreenLedToggle(void);

uint8 GPIO_SW1GetState(void);
uint8 GPIO_SW2GetState(void);

void GPIO_SW1EdgeTriggeredInterruptInit(void);
void GPIO_SW2EdgeTriggeredInterruptInit(void);
void GPIO_ExtraExternalButtonTriggeredInterruptInit(void);

#endif /* GPIO_H_ */
