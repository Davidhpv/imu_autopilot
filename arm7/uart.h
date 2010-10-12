/*=====================================================================

 PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

 (c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

 This file is part of the PIXHAWK project

 PIXHAWK is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 PIXHAWK is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

 ======================================================================*/

/**
* @file
*   @brief LPC2000 series UART driver
*
*   @author Martin Rutschmann <pixhawk@student.ethz.ch>
*   @author Dominik Honegger <pixhawk@student.ethz.ch>
*
*/

/*************************************************************************
 * This is the code for the UART connection for a LPC2148. It uses a
 * receive and a transmit buffer.
 * The sending is interrupt based. It fills the hardware uart fifo with
 * the data from the software buffer. If all bytes of the uart fifo has
 * been send out, there will an interrupt and the ISR will write the next
 * bytes from the send buffer into the hardware fifo.
 ************************************************************************/
#ifndef UART_H_
#define UART_H_

#include "conf.h"
#include "LPC21xx.h"


#define UART_BAUD(baud) (unsigned int)((PCLK / ((baud) * 16.0)) + 0.5)

/* Uart modes: */
#define UART_8N1      (unsigned char)(ULCR_CHAR_8 + ULCR_PAR_NO   + ULCR_STOP_1)
#define UART_7N1      (unsigned char)(ULCR_CHAR_7 + ULCR_PAR_NO   + ULCR_STOP_1)
#define UART_8N2      (unsigned char)(ULCR_CHAR_8 + ULCR_PAR_NO   + ULCR_STOP_2)
#define UART_7N2      (unsigned char)(ULCR_CHAR_7 + ULCR_PAR_NO   + ULCR_STOP_2)
#define UART_8E1      (unsigned char)(ULCR_CHAR_8 + ULCR_PAR_EVEN + ULCR_STOP_1)
#define UART_7E1      (unsigned char)(ULCR_CHAR_7 + ULCR_PAR_EVEN + ULCR_STOP_1)
#define UART_8E2      (unsigned char)(ULCR_CHAR_8 + ULCR_PAR_EVEN + ULCR_STOP_2)
#define UART_7E2      (unsigned char)(ULCR_CHAR_7 + ULCR_PAR_EVEN + ULCR_STOP_2)
#define UART_8O1      (unsigned char)(ULCR_CHAR_8 + ULCR_PAR_ODD  + ULCR_STOP_1)
#define UART_7O1      (unsigned char)(ULCR_CHAR_7 + ULCR_PAR_ODD  + ULCR_STOP_1)
#define UART_8O2      (unsigned char)(ULCR_CHAR_8 + ULCR_PAR_ODD  + ULCR_STOP_2)
#define UART_7O2      (unsigned char)(ULCR_CHAR_7 + ULCR_PAR_ODD  + ULCR_STOP_2)

/* FIFO modes: */
#define UART_FIFO_OFF (0x00)
#define UART_FIFO_1   (unsigned char)(UFCR_FIFO_ENABLE + UFCR_FIFO_TRIG1)
#define UART_FIFO_4   (unsigned char)(UFCR_FIFO_ENABLE + UFCR_FIFO_TRIG4)
#define UART_FIFO_8   (unsigned char)(UFCR_FIFO_ENABLE + UFCR_FIFO_TRIG8)
#define UART_FIFO_14  (unsigned char)(UFCR_FIFO_ENABLE + UFCR_FIFO_TRIG14)


/**
 * This function initializes the UART1.
 * @param baud The Baudrate for this UART port, (example baud=9200)
 * @param mode This parameter selects the mode of the port
 * @param fmode This parameter selects the FIFO mode
 */
void uart1_init( int baud, unsigned char mode, unsigned char fmode);

/**
 * This function checks if there is %len% free bytes in the transmit
 * buffer. This should be checked before using uart1_transmit.
 * @param len Number of bytes to check
 * @return 1 if there is enough free space, 0 if there isn't.
 */
int uart1_check_free_space( int len);

/**
 * Sends the byte data over the uart1. Before you use this function
 * check with uart1_check_free_space if there is enough place in the
 * transmit queue.
 * @param data the byte to send
 */
void uart1_transmit( unsigned char data );

/**
 * Check if there is data available in the uar1 receive queue.
 * @return 1 if there is data available, 0 if not.
 */
int uart1_char_available(void);

/**
 * Get the oldest byte of the uart1 receive queue. Before you use this
 * function, make sure with uart1_char_available if there is any data
 * in there is any data in the queue.
 * @return oldest byte in the receive queue
 */
unsigned char uart1_get_char(void);

/**
 * Get all bytes from the receive queue. Make sure the data array is
 * as big as the uart buffer.
 * @param data byte array in which the result will be written. It should
 * be as big as the uart buffer.
 * @param num The number of bytes which has been in the queue.
 */
void uart1_get_received_bytes(unsigned char* data,int* num);




//#ifdef DOWNLINK_USE_UART0
/**
 * This function initializes the UART0.
 * @param baud The Baudrate for this UART port, (example baud=9200)
 * @param mode This parameter selects the mode of the port
 * @param fmode This parameter selects the FIFO mode
 */
void uart0_init( int baud, unsigned char mode, unsigned char fmode);

/**
 * This function checks if there is %len% free bytes in the transmit
 * buffer. This should be checked before using uart0_transmit.
 * @param len Number of bytes to check
 * @return 1 if there is enough free space, 0 if there isn't.
 */
int uart0_check_free_space( int len);

/**
 * Sends the byte data over the uart0. Before you use this function
 * check with uart0_check_free_space if there is enough place in the
 * transmit queue.
 * @param data the byte to send
 */
void uart0_transmit( unsigned char data );

/**
 * Check if there is data available in the uar1 receive queue.
 * @return 1 if there is data available, 0 if not.
 */
int uart0_char_available(void);

/**
 * Get the oldest byte of the uart0 receive queue. Before you use this
 * function, make sure with uart0_char_available if there is any data
 * in there is any data in the queue.
 * @return oldest byte in the receive queue
 */
unsigned char uart0_get_char(void);

/**
 * Get all bytes from the receive queue. Make sure the data array is
 * as big as the uart buffer.
 * @param data byte array in which the result will be written. It should
 * be as big as the uart buffer.
 * @param num The number of bytes which has been in the queue.
 */
void uart0_get_received_bytes(unsigned char* data,int* num);
//#endif

#endif /* UART_H_ */
