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

#include "LPC21xx.h"
#include "uart.h"
#include "armVIC.h"
#include "led.h"
#include <string.h>


//#ifdef DOWNLINK_USE_UART1

void uart1_ISR(void) __attribute__((naked));

/* uart1 receive data queue */
unsigned char uart1_rx_buffer[UART1_RX_BUFFER_SIZE];
/* indexes for the receive buffer, insert_idx points to the next element to insert
 * and extract_idx points to the oldest unread element */
int uart1_rx_insert_idx, uart1_rx_extract_idx;

/* uart1 transmit data queue */
unsigned char uart1_tx_buffer[UART1_TX_BUFFER_SIZE];
/* indexes for the transmit buffer, insert_idx points to the next element to insert
 * and extract_idx points to the oldest unsent element */
int uart1_tx_insert_idx, uart1_tx_extract_idx;
/* uart1 transmit status, this is 1 if uart is sending and 0 if not */
int uart1_tx_running;

int uart1_check_free_space( int len) {
	/* calculate the number of free space between the tx_insert_idx and
	 * the tx_extract idx */
  int space = uart1_tx_extract_idx - uart1_tx_insert_idx;
  if (space <= 0)
    space += UART1_TX_BUFFER_SIZE;
  /* check if the free space is large or equal len */
  return (space - 1) >= len;
}

int uart1_char_available(void){
	/* if no char is available, uart1_rx_insert_idx would be equal to
	 * uart1_rx_extract_idx */
	return (uart1_rx_insert_idx != uart1_rx_extract_idx);
}

unsigned char uart1_get_char(void){
   unsigned char ret = uart1_rx_buffer[uart1_rx_extract_idx];
   uart1_rx_extract_idx = (uart1_rx_extract_idx + 1)%UART1_RX_BUFFER_SIZE;
   return ret;
}

void uart1_init( int baud, unsigned char mode, unsigned char fmode) {

	/* set port pins for UART1 */
	PINSEL0 = PINSEL0 | 0x00050000;

  U1IER = 0x00; // disable all interrupts
  U1IIR;        // clear interrupt ID
  U1RBR;        // clear receive register
  U1LSR;        // clear line status register

	/* calculate the divisor for set the correct baudrate */
	unsigned int uart_divisor = (PCLK / ((baud) * 16.0) + 0.5);
  /* set the divisor */
  U1LCR = ULCR_DLAB_ENABLE;                    // select divisor latches
  U1DLL = (unsigned char)uart_divisor;         // set for baud low byte
  U1DLM = (unsigned char)(uart_divisor >> 8);  // set for baud high byte

  /* set the number of characters */
  U1LCR = (mode & ~ULCR_DLAB_ENABLE);
  /* set other  user specified operating parameters */
  U1FCR = fmode;

  /* initialize the interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_UART1);                 // UART1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_UART1);                   // UART1 interrupt enabled
  _VIC_CNTL(UART1_VIC_SLOT) = VIC_ENABLE | VIC_UART1;
  _VIC_ADDR(UART1_VIC_SLOT) = (unsigned int)uart1_ISR; // address of the ISR

  /* initialize the transmit data queue */
  uart1_tx_extract_idx = 0;
  uart1_tx_insert_idx = 0;
  uart1_tx_running = 0;

  /* initialize the receive data queue */
  uart1_rx_extract_idx = 0;
  uart1_rx_insert_idx = 0;

  U1IER = UIER_ERBFI;					// enable receiver interrupts
}


void uart1_transmit( unsigned char data ) {
  int temp;
  unsigned cpsr;

  temp = (uart1_tx_insert_idx + 1) % UART1_TX_BUFFER_SIZE;  // calculate the next queue position

  if (temp == uart1_tx_extract_idx){	// check if there is free space in the send queue
    return;                          	// no room
  }

  cpsr = disableIRQ();                  // disable global interrupts
  U1IER &= ~UIER_ETBEI;                 // disable TX interrupts
  restoreIRQ(cpsr);                     // restore global interrupts

  if (uart1_tx_running)					// check if in process of sending data
    {
    uart1_tx_buffer[uart1_tx_insert_idx] = (unsigned char)data; // add data to queue
    uart1_tx_insert_idx = temp;			// increase insert pointer
    }
  else
    {
    uart1_tx_running = 1;				// set running flag
	U1THR = (unsigned char)data;		// write to output register
    }

  cpsr = disableIRQ();                  // disable global interrupts
  U1IER |= UIER_ETBEI;                  // enable TX interrupts
  restoreIRQ(cpsr);                     // restore global interrupts
}

void uart1_get_received_bytes(unsigned char* data,int* num){
	//copy current insert index
	int current_rx_insert_idx = uart1_rx_insert_idx;
	//calculate how many bytes are in the buffer
	*num = current_rx_insert_idx - uart1_rx_extract_idx;
	if (*num < 0){
		//in num is smaller than zero, you have to copy first the
		//end of the buffer array
		int end_num = UART1_RX_BUFFER_SIZE-uart1_rx_extract_idx;
		memcpy(data,uart1_rx_buffer+uart1_rx_extract_idx,end_num);
		//then copy the data at the begin of the uart buffer array
		memcpy(data+end_num,uart1_rx_buffer,current_rx_insert_idx);
		//calculate how many data has been copied
		*num += UART1_TX_BUFFER_SIZE;
	}
	else{
		//copy data from buffer
		memcpy(data,uart1_rx_buffer+uart1_rx_extract_idx,*num);
		//increase the uart1 extract index
		uart1_rx_extract_idx=current_rx_insert_idx;
	}
}


void uart1_ISR(void)
{
  ISR_ENTRY();

  int iid;

  while (((iid = U1IIR) & UIIR_NO_INT) == 0)  // loop until not more interrupt sources
    {
    switch (iid & UIIR_ID_MASK)			// identify & process the highest priority interrupt
      {
      case UIIR_RLS_INT:                // Receive Line Status
        U1LSR;                          // read LSR to clear
        break;

      case UIIR_CTI_INT:                // Character Timeout Indicator
      case UIIR_RDA_INT:                // Receive Data Available
    	  /* this do while loop will be running if a Charcter Timeout Indicator
    	   * or a Receive Data Available Interrupt has appeared */
        do
          {
          int temp;
          // calc next insert index & store character
          temp = (uart1_rx_insert_idx + 1) % UART1_RX_BUFFER_SIZE;
          uart1_rx_buffer[uart1_rx_insert_idx] = U1RBR;
          // check for more room in queue
          if (temp != uart1_rx_extract_idx)
            uart1_rx_insert_idx = temp; // update insert index
          }
        while (U1LSR & ULSR_RDR);

        break;

      case UIIR_THRE_INT:               // Transmit Holding Register Empty
        while (U1LSR & ULSR_THRE)
          {
          // check if more data to send
          if (uart1_tx_insert_idx != uart1_tx_extract_idx)
            {
            U1THR = uart1_tx_buffer[uart1_tx_extract_idx];
            uart1_tx_extract_idx++;
	    uart1_tx_extract_idx %= UART1_TX_BUFFER_SIZE;
            }
          else
            {
            // no
            uart1_tx_running = 0;       // clear running flag
            break;
            }
          }

        break;

      case UIIR_MS_INT:                 // MODEM Status
        U1MSR;                          // read MSR to clear
        break;

      default:                          // Unknown
        U1LSR;
        U1RBR;
        U1MSR;
        break;
      }
    }

  VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
  ISR_EXIT();                           // recover registers and return
}

//#endif




//#ifdef DOWNLINK_USE_UART0

void uart0_ISR(void) __attribute__((naked));

/* uart0 receive data queue */
unsigned char uart0_rx_buffer[UART0_RX_BUFFER_SIZE];
/* indexes for the receive buffer, insert_idx points to the next element to insert
 * and extract_idx points to the oldest unread element */
int uart0_rx_insert_idx, uart0_rx_extract_idx;

/* uart0 transmit data queue */
unsigned char uart0_tx_buffer[UART0_TX_BUFFER_SIZE];
/* indexes for the transmit buffer, insert_idx points to the next element to insert
 * and extract_idx points to the oldest unsent element */
int uart0_tx_insert_idx, uart0_tx_extract_idx;
/* uart0 transmit status, this is 1 if uart is sending and 0 if not */
int uart0_tx_running;

int uart0_check_free_space( int len) {
	/* calculate the number of free space between the tx_insert_idx and
	 * the tx_extract idx */
  int space = uart0_tx_extract_idx - uart0_tx_insert_idx;
  if (space <= 0)
    space += UART0_TX_BUFFER_SIZE;
  /* check if the free space is large or equal len */
  return (space - 1) >= len;
}

int uart0_char_available(void){
	/* if no char is available, uart0_rx_insert_idx would be equal to
	 * uart0_rx_extract_idx */
	return (uart0_rx_insert_idx != uart0_rx_extract_idx);
}

unsigned char uart0_get_char(void){
   unsigned char ret = uart0_rx_buffer[uart0_rx_extract_idx];
   uart0_rx_extract_idx = (uart0_rx_extract_idx + 1)%UART0_RX_BUFFER_SIZE;
   return ret;
}

void uart0_init( int baud, unsigned char mode, unsigned char fmode) {

	/* set port pins for uart0 */
	PINSEL0 = PINSEL0 | 0x0005;

  U0IER = 0x00; // disable all interrupts
  U0IIR;        // clear interrupt ID
  U0RBR;        // clear receive register
  U0LSR;        // clear line status register

	/* calculate the divisor for set the correct baudrate */
	unsigned int uart_divisor = (PCLK / ((baud) * 16.0) + 0.5);
  /* set the divisor */
  U0LCR = ULCR_DLAB_ENABLE;                    // select divisor latches
  U0DLL = (unsigned char)uart_divisor;         // set for baud low byte
  U0DLM = (unsigned char)(uart_divisor >> 8);  // set for baud high byte

  /* set the number of characters */
  U0LCR = (mode & ~ULCR_DLAB_ENABLE);
  /* set other  user specified operating parameters */
  U0FCR = fmode;

  /* initialize the interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_UART0);                 // uart0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_UART0);                   // uart0 interrupt enabled
  _VIC_CNTL(UART0_VIC_SLOT) = VIC_ENABLE | VIC_UART0;
  _VIC_ADDR(UART0_VIC_SLOT) = (unsigned int)uart0_ISR; // address of the ISR

  /* initialize the transmit data queue */
  uart0_tx_extract_idx = 0;
  uart0_tx_insert_idx = 0;
  uart0_tx_running = 0;

  /* initialize the receive data queue */
  uart0_rx_extract_idx = 0;
  uart0_rx_insert_idx = 0;

  U0IER = UIER_ERBFI;					// enable receiver interrupts
}


void uart0_transmit( unsigned char data ) {
  int temp;
  unsigned cpsr;

  temp = (uart0_tx_insert_idx + 1) % UART0_TX_BUFFER_SIZE;  // calculate the next queue position

  if (temp == uart0_tx_extract_idx){	// check if there is free space in the send queue
    return;                          	// no room
  }


  cpsr = disableIRQ();                  // disable global interrupts
  U0IER &= ~UIER_ETBEI;                 // disable TX interrupts
  restoreIRQ(cpsr);                     // restore global interrupts

  if (uart0_tx_running)					// check if in process of sending data
    {
    uart0_tx_buffer[uart0_tx_insert_idx] = (unsigned char)data; // add data to queue
    uart0_tx_insert_idx = temp;			// increase insert pointer
    }
  else
    {
    uart0_tx_running = 1;				// set running flag
	U0THR = (unsigned char)data;		// write to output register
    }

  cpsr = disableIRQ();                  // disable global interrupts
  U0IER |= UIER_ETBEI;                  // enable TX interrupts
  restoreIRQ(cpsr);                     // restore global interrupts
}

void uart0_get_received_bytes(unsigned char* data,int* num) {
	//copy current insert index
	int current_rx_insert_idx = uart0_rx_insert_idx;
	//calculate how many bytes are in the buffer
	*num = current_rx_insert_idx - uart0_rx_extract_idx;
	if (*num < 0) {
		//in num is smaller than zero, you have to copy first the
		//end of the buffer array
		int end_num = UART0_RX_BUFFER_SIZE-uart0_rx_extract_idx;
		memcpy(data, uart0_rx_buffer+uart0_rx_extract_idx, end_num);
		//then copy the data at the begin of the uart buffer array
		memcpy(data+end_num, uart0_rx_buffer, current_rx_insert_idx);
		//calculate how many data has been copied
		*num += UART0_TX_BUFFER_SIZE;
	}
	else {
		//copy data from buffer
		memcpy(data, uart0_rx_buffer+uart0_rx_extract_idx, *num);
		//increase the uart0 extract index
		uart0_rx_extract_idx=current_rx_insert_idx;
	}
}

void uart0_ISR(void)
{
  ISR_ENTRY();

  int iid;

  while (((iid = U0IIR) & UIIR_NO_INT) == 0)  // loop until not more interrupt sources
    {
    switch (iid & UIIR_ID_MASK)			// identify & process the highest priority interrupt
      {
      case UIIR_RLS_INT:                // Receive Line Status
        U0LSR;                          // read LSR to clear
        break;

      case UIIR_CTI_INT:                // Character Timeout Indicator
      case UIIR_RDA_INT:                // Receive Data Available
    	  /* this do while loop will be running if a Charcter Timeout Indicator
    	   * or a Receive Data Available Interrupt has appeared */
        do
          {
          int temp;
          // calc next insert index & store character
          temp = (uart0_rx_insert_idx + 1) % UART0_RX_BUFFER_SIZE;
          uart0_rx_buffer[uart0_rx_insert_idx] = U0RBR;
          // check for more room in queue
          if (temp != uart0_rx_extract_idx)
            uart0_rx_insert_idx = temp; // update insert index
          }
        while (U0LSR & ULSR_RDR);

        break;

      case UIIR_THRE_INT:               // Transmit Holding Register Empty
        while (U0LSR & ULSR_THRE)
          {
          // check if more data to send
          if (uart0_tx_insert_idx != uart0_tx_extract_idx)
            {
            U0THR = uart0_tx_buffer[uart0_tx_extract_idx];
            uart0_tx_extract_idx++;
	    uart0_tx_extract_idx %= UART0_TX_BUFFER_SIZE;
            }
          else
            {
            // no
            uart0_tx_running = 0;       // clear running flag
            break;
            }
          }

        break;

      default:                          // Unknown
        U0LSR;
        U0RBR;
        break;
      }
    }

  VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
  ISR_EXIT();                           // recover registers and return
}

//#endif
