/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Martin Rutschmann
Contributing Authors (in alphabetical order):
  Dominik Honegger



(c) 2008, 2009 PIXHAWK PROJECT

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
#include "uart.h"
#include "downlink.h"

void downlink_init(void){

#ifdef DOWNLINK_USE_UART0
	uart0_init( DOWNLINK_BAUD, DOWNLINK_MODE, UART_FIFO_8);
#endif

#ifdef DOWNLINK_USE_UART1
	uart1_init( DOWNLINK_BAUD, DOWNLINK_MODE, UART_FIFO_8);
#endif

}


void downlink_send(unsigned char byte){

#ifdef DOWNLINK_USE_UART0
	uart0_transmit(byte);
#endif

#ifdef DOWNLINK_USE_UART1
	uart1_transmit(byte);
#endif

}


int downlink_char_available(void){

#ifdef DOWNLINK_USE_UART0
	return uart0_char_available();
#endif

#ifdef DOWNLINK_USE_UART1
	return uart1_char_available();
#endif

}


unsigned char downlink_get_byte(void){

#ifdef DOWNLINK_USE_UART0
	return uart0_get_char();
#endif

#ifdef DOWNLINK_USE_UART1
	return uart1_get_char();
#endif

}
