/**************************************************************************//**
 * @file     system_ARMCM0.c
 * @brief    CMSIS ARM Cortex-M0 Device Peripheral Access Layer Source File
 * @version  V1.02
 * @date     08. September 2009
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


#include <stdint.h>
#include "armcm0.h"

/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/
#define __HSI (8000000UL)

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
//uint32_t SystemCoreClock  = __HSI;   /*!< System Clock Frequency (Core Clock) */
uint32_t SystemCoreClock  = 72000000UL;   /*!< System Clock Frequency (Core Clock) */


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
  /* Determine clock frequency according to clock register values             */

  /* NEEDS TO BE IMPLEMENTED */

}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemFrequency variable.
 */
void SystemInit (void)
{
  GPIO0->DATA[0].WORD = 0;
  GPIO0->IE = 0;
  GPIO0->DIR = 0xff83;
  
  GPIO1->DATA[0].WORD = 0;
  GPIO1->IE = 0;
  GPIO1->DIR = 0;
  
  GPIO2->DATA[0].WORD = 0;
  GPIO2->IE = 0;
  GPIO2->DIR = 0;
}

