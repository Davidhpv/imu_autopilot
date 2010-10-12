/**************************************************************************//**
 * @file     main.c
 * @brief    CMSIS Cortex-M3 Blinky example
 *           Blink a LED using CM3 SysTick
 * @version  V1.03
 * @date     24. September 2009
 *
 * @note
 * Copyright (C) 2009 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

#include <stdlib.h>
#include "AT91SAM3U4.h"                               /* Device Series Header */


volatile uint32_t msTicks;                            /* counts 1ms timeTicks */
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;                        /* increment counter necessary in Delay() */
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
__INLINE static void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}

/*----------------------------------------------------------------------------
  configer LED pins
 *----------------------------------------------------------------------------*/
__INLINE static void LED_Config(void) {

  PIOB->PIO_PER    =  0x01;          /* Setup Pin PB0 for LED */
  PIOB->PIO_OER    =  0x01;
  PIOB->PIO_PUDR   =  0x01;
}

/*----------------------------------------------------------------------------
  Switch on LEDs
 *----------------------------------------------------------------------------*/
__INLINE static void LED_On (uint32_t led) {

  PIOB->PIO_CODR = led;              /* Turn On  LED */
}


/*----------------------------------------------------------------------------
  Switch off LEDs
 *----------------------------------------------------------------------------*/
__INLINE static void LED_Off (uint32_t led) {

  PIOB->PIO_SODR = led;              /* Turn Off LED */
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {

  if (SysTick_Config(SystemCoreClock / 1000)) { /* Setup SysTick Timer for 1 msec interrupts  */
    while (1);                                  /* Capture error */
  }
  
  LED_Config();                             
 
  while(1) {
    LED_On (0x01);                              /* Turn on the LED. */
    Delay (100);                                /* delay  100 Msec */
    LED_Off (0x01);                             /* Turn off the LED. */
    Delay (100);                                /* delay  100 Msec */
  }
  
}
