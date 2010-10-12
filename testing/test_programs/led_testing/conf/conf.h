/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Martin Rutschmann <mavteam@student.ethz.ch>
  Dominik Honegger <mavteam@student.ethz.ch>
Contributing Authors (in alphabetical order):
  Lorenz Meier <lm@student.ethz.ch>

Todo:


(c) 2009 PIXHAWK PROJECT

This file is part of the PIXHAWK project

    mavlib is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    mavlib is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with mavlib. If not, see <http://www.gnu.org/licenses/>.

========================================================================*/

#ifndef CONF_H_
#define CONF_H_



//############### GENERAL SETUP #####################
#define STX 0xAA
#define ACID 120
#define ML_PAYLOAD_LEN 255


/* Master oscillator freq.       */
#define FOSC (12000000)
/* PLL multiplier                */
#define PLL_MUL (5)
/* CPU clock freq.               */
#define CCLK (FOSC * PLL_MUL)
/* Peripheral bus speed mask 0x00->4, 0x01-> 1, 0x02 -> 2   */
#define PBSD_BITS 0x00
#define PBSD_VAL 4
/* Peripheral bus clock freq.    */
#define PCLK (CCLK / PBSD_VAL)


/*LED***********************************************************************************/
/** @name LED
 *  Pin settings for the LEDs. */
//@{
/*! LED pin. This is the pin on witch the led is connected. */
#define LED_GREEN			22
/*! GPIO direction register for LED (IO0DIR or IO1DIR) */
#define LED_GREEN_DIR		IO1DIR
/*! GPIO clear register for LED (IO0CLR or IO1CLR) */
#define LED_GREEN_CLEAR		IO1CLR
/*! GPIO set register for LED (IO0SET or IO1SET) */
#define LED_GREEN_SET		IO1SET
/*! GPIO pin register for LED (IO0PIN or IO1PIN) */
#define LED_GREEN_PIN		IO1PIN

/*! LED pin. This is the pin on witch the led is connected. */
#define LED_RED			23
/*! GPIO direction register for LED (IO0DIR or IO1DIR) */
#define LED_RED_DIR		IO1DIR
/*! GPIO clear register for LED (IO0CLR or IO1CLR) */
#define LED_RED_CLEAR	IO1CLR
/*! GPIO set register for LED (IO0SET or IO1SET) */
#define LED_RED_SET		IO1SET
/*! GPIO pin register for LED (IO0PIN or IO1PIN) */
#define LED_RED_PIN		IO1PIN



#define PERIODIC_TASK_PERIOD SYS_TICS_OF_SEC( 20e-3 )

/* Interrupt Vector Slots, Priority: 1=highest ***************************************/

#define TIMER0_VIC_SLOT 2
#define UART1_VIC_SLOT 5
#define UART0_VIC_SLOT 6
#define MICROMAG_DRDY_VIC_SLOT 7
#define MAX1168_EOC_VIC_SLOT 8
#define SSP_VIC_SLOT 9
#define SCP1000_EOC_VIC_SLOT 10

#endif /* CONF_H_ */
