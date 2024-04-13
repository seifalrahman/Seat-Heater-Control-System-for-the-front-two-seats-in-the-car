/*
 * adc.h
 *
 *  Created on: Apr 8, 2024
 *      Author: seif alrahman
 */

#ifndef MCAL_ADC_ADC_H_
#define MCAL_ADC_ADC_H_

#include "std_types.h"
/////////////////////___PLL____///////////////////////////
#define SYSCTL_RCC2_USERCC2_MASK        0x80000000  /* USERCC2 Bit MASK */
#define SYSCTL_RCC2_BYPASS2_MASK        0x00000800  /* PLL BYPASS2 Bit MASK */
#define SYSCTL_RCC_XTAL_MASK            0x000007C0  /* XTAL Bits MASK */
#define SYSCTL_RCC_XTAL_8MHZ            0x0E        /* 8 MHz Crystal Value */
#define SYSCTL_RCC_XTAL_16MHZ           0x15        /* 16 MHz Crystal Value */
#define SYSCTL_RCC_XTAL_BIT_POS         6           /* XTAL Bits Position start from bit number 6 */
#define SYSCTL_RCC2_OSCSRC2_MASK        0x00000070  /* OSCSRC2 Bits MASK */
#define SYSCTL_RCC2_OSCSRC2_MOSC        0x0         /* MOSC(Main Oscillator) value */
#define SYSCTL_RCC2_OSCSRC2_BIT_POS     4           /* OSCSRC2 Bits Position start from bit number 4 */
#define SYSCTL_RCC2_PWRDN2_MASK         0x00002000  /* PWRDN2 Bit MASK */
#define SYSCTL_RCC2_DIV400_MASK         0x40000000  /* DIV400 Bit MASK to Divide PLL as 400 MHz vs. 200 */
#define SYSCTL_RCC2_SYSDIV2_MASK        0x1FC00000  /* SYSDIV2 Bits MASK */
#define SYSCTL_RIS_PLLLRIS_MASK         0x00000040  /* PLLLRIS Bit MASK */
#define SYSCTL_RCC2_SYSDIV2_BIT_POS     22       /* SYSDIV2 Bits Position start from bit number 22 */
#define SYSDIV2_VALUE                   4
//////////////////////////////////////////////////////////

//ADC0 module
#define ADC0_ADCACTSS (*((volatile unsigned long*)0x40038000))
#define ADC0_ADCEMUX (*((volatile unsigned long*)0x40038014))
#define ADC0_ADCSSMUX3 (*((volatile unsigned long*)0x400380A0))
#define ADC0_ADCSSCTL3 (*((volatile unsigned long*)0x400380A4))
#define ADC0_ADCPSSI (*((volatile unsigned long*)0x40038028))
#define ADC0_ADCISC (*((volatile unsigned long*)0x4003800C))
#define ADC0_ADCSSFIFO3 (*((volatile unsigned long*)0x400380A8))
#define ADC0_ADCSSFIFO3 (*((volatile unsigned long*)0x400380A8))
#define ADC0_ADCRIS (*((volatile unsigned long*)0x40038004))
#define ADC0_ADCIM  (*((volatile unsigned long*)0x40038008))

//*******************************************************************************************************************
//ADC1 module
#define ADC1_ADCACTSS (*((volatile unsigned long*)0x40039000))
#define ADC1_ADCEMUX (*((volatile unsigned long*)0x40039014))
#define ADC1_ADCSSMUX3 (*((volatile unsigned long*)0x400390A0))
#define ADC1_ADCSSCTL3 (*((volatile unsigned long*)0x400390A4))
#define ADC1_ADCPSSI (*((volatile unsigned long*)0x40039028))
#define ADC1_ADCISC (*((volatile unsigned long*)0x4003900C))
#define ADC1_ADCSSFIFO3 (*((volatile unsigned long*)0x400390A8))
#define ADC1_ADCRIS (*((volatile unsigned long*)0x40039004))
#define ADC1_ADCIM  (*((volatile unsigned long*)0x40039008))


//*******************************************************************************************************************



#define ADC_MODULE_0_ENABLE  (1<<0)
#define ADC_MODULE_1_ENABLE  (1<<1)







void ADCModuleInitialization (void);
void sampleSequencerConfiguration (void);
uint32 ADC1_readChannel (void);
uint32 ADC0_readChannel (void);
void ADC_INIT(void);
void PLL_Init(void);

#endif /* MCAL_ADC_ADC_H_ */
