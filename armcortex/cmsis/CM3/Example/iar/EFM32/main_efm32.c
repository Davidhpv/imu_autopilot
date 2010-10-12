/*************************************************************************//**
 * @file:    main_efm32.c
 * @purpose: CMSIS Cortex-M3 Core Peripheral Access Layer Source File
 *           Basic SysTick interrupt
 * @version 1.1.1
 *----------------------------------------------------------------------------
 *
 * Copyright (C) 2009 ARM Limited. All rights reserved.
 *
 * ARM Limited (ARM) is supplying this software for use with Cortex-M3
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *****************************************************************************/

#include "efm32.h"
#include "dvkreg.h"

volatile uint32_t msTicks;                        /**< Counts 1ms timeTicks */
/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;                        /* increment counter necessary in Delay()*/
}

/**************************************************************************//**
 * @brief Delays number of msTick Systicks (happens every 1 ms)
 * @param uint32_t dlyTicks
 *  Number of ticks to delay
 *****************************************************************************/
__INLINE static void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  SystemCoreClockUpdate();
  if (SysTick_Config(SystemCoreClock / 1000))   /* Setup SysTick Timer for 1 msec interrupts  */
  {
    while (1) ;                                 /* Capture error */
  }

  /* Initialize DVK board register access */
  DVK_init();

  while (1)
  {
    /* Enable user leds on DVK board */
    DVK_writeRegister( &DVK_REGISTER->UIF_LEDS, 0xffff);
    /* Busy wait 0.5 sec */
    Delay(500); 
    /* Disable user leds on DVK board */
    DVK_writeRegister( &DVK_REGISTER->UIF_LEDS, 0x0000);
    Delay(500); 
  }
}
