/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Dominik Honegger
  Tobias Naegeli
  Martin Rutschmann
Contributing Authors (in alphabetical order):



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

#include "LPC21xx.h"
#include "mcu_init.h"
#include "led.h"
#include "downlink.h"
#include "armVIC.h"
#include "sys_time.h"
#include "comm_temp.h"

void main_init (void);
void main_periodic(void);
void dac_set (unsigned int value);

void main_init (void){
	hw_init();
	led_init();
	enableIRQ();
	sys_time_init();
	downlink_init();

	led_on(LED_GREEN);
}

#define DAC_VALUE_MASK	(0x3FF)
#define DAC_PINSEL		PINSEL1
#define DAC_PINSEL_VAL	2
#define DAC_PINSEL_BIT	18

void adc_init (void)
{
	//infrared
	//PINSEL1 |= (1<<26); // set function P0.29 as AC0.2

	//int CLKDIV = (int) (PCLK / 4500000);
	//if(CLKDIV>0){
	//	CLKDIV=CLKDIV-1;
	//}
	//AD0CR = ( 1<<2 | 1<<16 | CLKDIV<<8 | 1<<21);

	//bat
	//PINSEL0 |= (3<<30); // set function P0.15 as AC1.5

	//AD1CR = ( 1<<5 | 1<<16 | CLKDIV<<8 | 1<<21);
}

void dac_init (void){
	DAC_PINSEL|=DAC_PINSEL_VAL<<DAC_PINSEL_BIT;
	DACR = 1<<16; //set output in powermode and set output to 0V
}

void dac_set (unsigned int value){
	value&=DAC_VALUE_MASK;
	DACR = value<<6; // bits 0-5 are reserved
}

void main_periodic(void){
	dac_init();
	sys_time_periodic_init();
	for(int i=0; i<50; i++){
		while(sys_time_periodic()!=1);
	}
	while(sys_time_periodic()!=1);
	while(1){
		while(sys_time_periodic()!=1);

		static unsigned int val=0;
		led_toggle(LED_RED);
		if (downlink_char_available()){
			unsigned char read=downlink_get_byte();
			if(read=='+'){
				dac_set(++val);
				char buffer[100];
				comm_int_to_string(val,buffer);
				comm_send_string(buffer);
				comm_send_string("\r\n");
			}
			if(read=='-'){
				dac_set(--val);
				char buffer[100];
				comm_int_to_string(val,buffer);
				comm_send_string(buffer);
				comm_send_string("\r\n");
			}
		}
	}
}



int main(void) {
	main_init();
	main_periodic();
	return 0;
}
