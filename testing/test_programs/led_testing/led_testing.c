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

	led_on(LED_GREEN);
}


void main_periodic(void){
	sys_time_periodic_init();
	for(int i=0; i<50; i++){
		while(sys_time_periodic()!=1);
	}
	while(1){
		while(sys_time_periodic()!=1);
		led_toggle(LED_RED);
	}
}



int main(void) {
	main_init();
	main_periodic();
	return 0;
}
