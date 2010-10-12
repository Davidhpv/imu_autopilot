/******************************************************************************
 * @file:    system_TMPM330.c
 * @purpose: CMSIS Cortex-M3 Device Peripheral Access Layer Source File for the
 *           TOSHIBA 'TMPM330' Device Series 
 * @version: V2.0.0
 * @date:    2009/10/14
 * 
 * THE SOURCE CODE AND ITS RELATED DOCUMENTATION IS PROVIDED "AS IS". TOSHIBA
 * CORPORATION MAKES NO OTHER WARRANTY OF ANY KIND, WHETHER EXPRESS, IMPLIED OR,
 * STATUTORY AND DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * SATISFACTORY QUALITY, NON INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 * 
 * THE SOURCE CODE AND DOCUMENTATION MAY INCLUDE ERRORS. TOSHIBA CORPORATION
 * RESERVES THE RIGHT TO INCORPORATE MODIFICATIONS TO THE SOURCE CODE IN LATER
 * REVISIONS OF IT, AND TO MAKE IMPROVEMENTS OR CHANGES IN THE DOCUMENTATION OR
 * THE PRODUCTS OR TECHNOLOGIES DESCRIBED THEREIN AT ANY TIME.
 * 
 * TOSHIBA CORPORATION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGE OR LIABILITY ARISING FROM YOUR USE OF THE SOURCE CODE OR
 * ANY DOCUMENTATION, INCLUDING BUT NOT LIMITED TO, LOST REVENUES, DATA OR
 * PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL OR CONSEQUENTIAL NATURE, PUNITIVE
 * DAMAGES, LOSS OF PROPERTY OR LOSS OF PROFITS ARISING OUT OF OR IN CONNECTION
 * WITH THIS AGREEMENT, OR BEING UNUSABLE, EVEN IF ADVISED OF THE POSSIBILITY OR
 * PROBABILITY OF SUCH DAMAGES AND WHETHER A CLAIM FOR SUCH DAMAGE IS BASED UPON
 * WARRANTY, CONTRACT, TORT, NEGLIGENCE OR OTHERWISE.
 * 
 * (C)Copyright TOSHIBA CORPORATION 2009 All rights reserved
 ******************************************************************************/

#include <stdint.h>
#include "TMPM330.h"

/*-------- <<< Start of configuration section >>> ----------------------------*/

/* Watchdog Timer (WD) Configuration */
#define WD_SETUP    1
#define WDMOD_Val   0x00000000

/* Clock Generator (CG) Configuration */
#define CLOCK_SETUP 1
#define SYSCR_Val   0x00010000
#define OSCCR_Val   0x00000314
#define STBYCR_Val  0x00000103
#define PLLSEL_Val  0x00000001
#define CKSEL_Val   0x00000000

/*-------- <<< End of configuration section >>> ------------------------------*/

/*-------- DEFINES -----------------------------------------------------------*/
/* Define clocks */
#define XTALH (10000000UL)  /* External high-speed oscillator freq */
#define XTALL (   32768UL)  /* External low-speed oscillator freq  */

/* Determine core clock frequency according to settings */
  #if (CKSEL_Val & (1 << 1))            /* If system clock is low-speed clock */
    __CORE_CLK = XTALL;
  #else                                 /* If system clock is high-speed clock*/
    #if ((PLLSEL_Val & (1<<0)) && (OSCCR_Val & (1<<2))) /* If PLL selected and enabled */
      #if   ((SYSCR_Val & 7) == 0)      /* Gear -> fc                         */
          #define __CORE_CLK   (XTALH * 4)
      #elif ((SYSCR_Val & 7) == 4)      /* Gear -> fc/2                       */
          #define __CORE_CLK   (XTALH * 4 / 2)
      #elif ((SYSCR_Val & 7) == 5)      /* Gear -> fc/4                       */
          #define __CORE_CLK   (XTALH * 4 / 4)
      #elif ((SYSCR_Val & 7) == 6)      /* Gear -> fc/8                       */
          #define __CORE_CLK   (XTALH * 4 / 8)
      #else                             /* Gear -> reserved                   */
          #define __CORE_CLK   (XTALH)
      #endif
    #else
      #if   ((SYSCR_Val & 7) == 0)      /* Gear -> fc                         */
          #define __CORE_CLK   (XTALH * 4)
      #elif ((SYSCR_Val & 7) == 4)      /* Gear -> fc/2                       */
          #define __CORE_CLK   (XTALH * 4 / 2)
      #elif ((SYSCR_Val & 7) == 5)      /* Gear -> fc/4                       */
          #define __CORE_CLK   (XTALH * 4 / 4)
      #elif ((SYSCR_Val & 7) == 6)      /* Gear -> fc/8                       */
          #define __CORE_CLK   (XTALH * 4 / 8)
      #else                             /* Gear -> reserved                   */
          #define __CORE_CLK   (XTALH)
      #endif
    #endif                              /* If PLL not used                    */
  #endif

/* Clock Variable definitions */
uint32_t SystemCoreClock = __CORE_CLK;/*!< System Clock Frequency (Core Clock)*/


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Update SystemCoreClock according register values.
 */
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
  /* Determine clock frequency according to clock register values        */
  if (CG_CKSEL_SYSCK) {           /* If system clock is low-speed clock  */
    SystemCoreClock = XTALL;
  } else {                        /* If system clock is high-speed clock */
    if (CG_PLLSEL_PLLSEL && CG_OSCCR_PLLON) {   /* If PLL enabled      */
      switch (CG->SYSCR & 7) {
        case 0:                                 /* Gear -> fc          */
          SystemCoreClock = XTALH * 4;
          break;
        case 1:
        case 2:
        case 3:
        case 7:                                 /* Gear -> reserved    */
          SystemCoreClock = XTALH;
          break;
        case 4:                                 /* Gear -> fc/2        */
          SystemCoreClock = XTALH * 4 / 2;
          break;
        case 5:                                 /* Gear -> fc/4        */
          SystemCoreClock = XTALH * 4 / 4;
          break;
        case 6:                                 /* Gear -> fc/8        */
          SystemCoreClock = XTALH * 4 / 8;
          break;
      }
    } else {
      switch (CG->SYSCR & 7) {
        case 0:                                 /* Gear -> fc          */
          SystemCoreClock = XTALH;
          break;
        case 1:
        case 2:
        case 3:
        case 7:                                 /* Gear -> reserved    */
          SystemCoreClock = XTALH;
          break;
        case 4:                                 /* Gear -> fc/2        */
          SystemCoreClock = XTALH / 2;
          break;
        case 5:                                 /* Gear -> fc/4        */
          SystemCoreClock = XTALH / 4;
          break;
        case 6:                                 /* Gear -> fc/8        */
          SystemCoreClock = XTALH / 8;
          break;
      }
    }
  }
}
/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit(void)
{
#if (WD_SETUP)                  /* Watchdog Setup */
  WD->MOD = WDMOD_Val;
  if (!WD_MOD_WDTE) {           /* If watchdog is to be disabled */
    WD->CR = 0xB1;
  }
#endif

#if (CLOCK_SETUP)               /* Clock Setup */
  CG->SYSCR  = SYSCR_Val;
  CG->OSCCR  = OSCCR_Val;
  CG->STBYCR = STBYCR_Val;
  CG->PLLSEL = PLLSEL_Val;
  if (CG_PLLSEL_PLLSEL && CG_OSCCR_PLLON) {   /* If PLL enabled */
    CG_OSCCR_WUEON = 1;
    while(CG_OSCCR_WUEF != 0){}               /* Warm-up        */
  }
  CG->CKSEL  = CKSEL_Val;
#endif

}
