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
#include "adc.h"
#include "comm_temp.h"
#include "scp1000.h"

void main_init (void);
void main_periodic(void);

void main_init (void){
	hw_init();
	led_init();
	enableIRQ();
	sys_time_init();
	downlink_init();
	scp1000_init();

	led_on(LED_GREEN);
}

void main_periodic(void){
	sys_time_periodic_init();
	for(int i=0; i<50; i++){
		while(sys_time_periodic()!=1);
	}
	while(sys_time_periodic()!=1);
	while(1){
		while(sys_time_periodic()!=1);

		led_toggle(LED_RED);
		if (downlink_char_available()){
			unsigned char read=downlink_get_byte();
			if(read=='a'){
				scp1000_send_config(SCP_MEAS_TRIG);
				downlink_send(read);
			}
			if(read=='b'){
				scp1000_send_config(SCP_MEAS_STOP);
				downlink_send(read);
			}
			if(read=='c'){
				downlink_send(read);
				char buffer[100];
			    comm_send_string(" pressure_raw : ");
			    int test=scp1000_get_value();
				comm_int_to_string(test,buffer);
				comm_send_string(buffer);
				comm_send_string("\r\n");
			}
			if(read=='d'){
				scp1000_read();
				downlink_send(read);
			}
			if(read=='e'){
				scp1000_read_statusreg();
				downlink_send(read);
			}
			if(read=='f'){
				downlink_send(read);
				char buffer[100];
			    comm_send_string(" status_reg : ");
			    int test=scp1000_get_statusreg();
				comm_int_to_string(test,buffer);
				comm_send_string(buffer);
				comm_send_string("\r\n");
			}
			if(read=='i'){
				scp1000_send_config(SCP_MEAS_INIT);
				downlink_send(read);
			}
			if(read=='s'){
				scp1000_send_config(SCP_MEAS_SELFTEST);
				downlink_send(read);
			}
			if(read==' '){
				scp1000_send_software_reset();
				downlink_send(read);
			}
		}
	}
}

int main(void) {
	main_init();
	main_periodic();
	return 0;
}
