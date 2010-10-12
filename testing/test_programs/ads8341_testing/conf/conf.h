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

// P1.17 (as defined by using IO1DIR/IO1SET and number 17)
#define ADS8341_SS_PIN   17


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
#define LED_YELLOW	16
#define LED_GREEN	22
#define LED_RED		23

#define LED_YELLOW_PINSEL PINSEL2
#define LED_YELLOW_PINSEL_VAL 0x01
#define LED_YELLOW_PINSEL_BIT 3

#define LED_RED_PINSEL PINSEL2
#define LED_RED_PINSEL_VAL 0x01
#define LED_RED_PINSEL_BIT 3

#define LED_GREEN_PINSEL PINSEL2
#define LED_GREEN_PINSEL_VAL 0x01
#define LED_GREEN_PINSEL_BIT 3

#define LED_DIR IO1DIR
#define LED_CLEAR IO1CLR
#define LED_SET IO1SET
#define LED_PIN IO1PIN


/*UART*********************************************************************************/

#define UART0_RX_BUFFER_SIZE 512        // UART0 receive buffer size
#define UART0_TX_BUFFER_SIZE 512        // UART0 transmit buffer size

#define UART1_RX_BUFFER_SIZE 512        // UART1 receive buffer size
#define UART1_TX_BUFFER_SIZE 512        // UART1 transmit buffer size

/*Downlink**********************************************************************************/

#define DOWNLINK_USE_UART0
#define DOWNLINK_BAUD 		115200
#define DOWNLINK_MODE		UART_8N1

/* Periodic events ***************************************/

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
