/*
 * adc.c
 *
 *  Created on: Apr 8, 2024
 *      Author: seif alrahman
 */

#include "gpio.h"
#include "adc.h"
#include "tm4c123gh6pm_registers.h"


volatile static uint32 adc0Result =0 ;
volatile static uint32 adc1Result =0 ;
void ADC_INIT(void){
    ADCModuleInitialization();
    sampleSequencerConfiguration();
}

void ADC0SS3_handler(void){
    adc0Result=ADC0_ADCSSFIFO3 ;
    ADC0_ADCISC= (1<<3)       ;

}
void ADC1SS3_handler(void){
    adc1Result  = ADC1_ADCSSFIFO3 ;
    ADC1_ADCISC = (1<<3)          ;
}

void ADCModuleInitialization (void){
    PLL_Init();
    //Enable ADC modules
    SYSCTL_RCGCADC_REG|=ADC_MODULE_0_ENABLE ;
    SYSCTL_RCGCADC_REG|=ADC_MODULE_1_ENABLE ;

    SYSCTL_RCGCGPIO_REG|=(1<<4) ;
    GPIO_PORTE_AFSEL_REG|=((1<<2)|(1<<3)) ;
    /*Configure the AINx signals to be analog inputs by clearing the corresponding DEN bit in the
     *GPIO Digital Enable (GPIODEN) register (see page 682).
     *
     * */
    GPIO_PORTE_DEN_REG &=~ ((1<<2)|(1<<3)) ;
    /*Disable the analog isolation circuit for all ADC input pins that are to be used by writing a 1 to
    the appropriate bits of the GPIOAMSEL register (see page 687) in the associated GPIO block.*/
    GPIO_PORTE_AMSEL_REG|=((1<<3)|(1<<2)) ;
    //don't change
    /*If required by the application, reconfigure the sample sequencer priorities in the ADCSSPRI
    register. The default configuration has Sample Sequencer 0 with the highest priority and Sample
    Sequencer 3 as the lowest priority.
     */

}

void sampleSequencerConfiguration (void){
    ADC0_ADCACTSS =0;
    ADC1_ADCACTSS =0;
    /**/
    ADC1_ADCEMUX  =(0xF<<12);
    ADC0_ADCEMUX  =(0xF<<12);
    /*4. For each sample in the sample sequence, configure the corresponding input source in the
    ADCSSMUXn register.
     * */
    ADC0_ADCSSMUX3 =0;
    ADC1_ADCSSMUX3 =1;

    /*5. For each sample in the sample sequence, configure the sample control bits in the corresponding
    nibble in the ADCSSCTLn register. When programming the last nibble, ensure that the END bit
    is set. Failure to set the END bit causes unpredictable behavior.
     *
     * */

    ADC0_ADCSSCTL3|=( (1<<1) | (1<<2) );
    ADC1_ADCSSCTL3|=( (1<<1) | (1<<2) );

    /*
     * 6. If interrupts are to be used, set the corresponding MASK bit in the ADCIM register.
     * */
    ADC0_ADCIM|= (1<<3) ;
    ADC1_ADCIM|= (1<<3) ;
    /* Enable the sample sequencer logic by setting the corresponding ASENn bit in the ADCACTSS
     *register.
     */
    ADC0_ADCACTSS = (1<<3) ;
    ADC1_ADCACTSS = (1<<3) ;

    //clear status bit by writing one into he corresponding bit position
    ADC0_ADCISC= (1<<3) ;
    ADC1_ADCISC= (1<<3) ;

}

uint32 ADC0_readChannel (void){

    return  adc0Result ;
}

uint32 ADC1_readChannel (void){

    return  adc1Result ;
}

/* configure the system to get its clock from the PLL with Frequency 80Mhz */
void PLL_Init(void)
{
    /* 1) Configure the system to use RCC2 for advanced features
          such as 400 MHz PLL and non-integer System Clock Divisor */
    SYSCTL_RCC2_REG |= SYSCTL_RCC2_USERCC2_MASK;

    /* 2) Bypass PLL while initializing, Dont use PLL while initialization */
    SYSCTL_RCC2_REG |= SYSCTL_RCC2_BYPASS2_MASK;

    /* 3) Select the crystal value and oscillator source */
    SYSCTL_RCC_REG  &= ~SYSCTL_RCC_XTAL_MASK;     /* clear XTAL field */
    SYSCTL_RCC_REG  |= (SYSCTL_RCC_XTAL_16MHZ << SYSCTL_RCC_XTAL_BIT_POS);  /* Set the XTAL bits for 16 MHz crystal */

    SYSCTL_RCC2_REG &= ~SYSCTL_RCC2_OSCSRC2_MASK; /* clear oscillator source field (OSCSRC2 bits) */
    SYSCTL_RCC2_REG |= (SYSCTL_RCC2_OSCSRC2_MOSC << SYSCTL_RCC2_OSCSRC2_BIT_POS);  /* configure for main oscillator source */

    /* 4) Activate PLL by clearing PWRDN2 */
    SYSCTL_RCC2_REG &= ~SYSCTL_RCC2_PWRDN2_MASK;

    /* 5) Set the desired system divider and the system divider least significant bit */
    SYSCTL_RCC2_REG |= SYSCTL_RCC2_DIV400_MASK;  /* use 400 MHz PLL */

    SYSCTL_RCC2_REG  = (SYSCTL_RCC2_REG & ~SYSCTL_RCC2_SYSDIV2_MASK)        /* clear system clock divider field */
                       | (SYSDIV2_VALUE << SYSCTL_RCC2_SYSDIV2_BIT_POS);      /* configure for 80MHz clock */

    /* 6) Wait for the PLL to lock by polling PLLLRIS bit */
    while(!(SYSCTL_RIS_REG & SYSCTL_RIS_PLLLRIS_MASK));

    /* 7) Enable use of PLL by clearing BYPASS2 */
    SYSCTL_RCC2_REG &= ~SYSCTL_RCC2_BYPASS2_MASK;
}



