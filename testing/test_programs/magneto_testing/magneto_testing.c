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
#include "ms2100.h"
#include <math.h>

void main_init (void);
void main_periodic(void);
short y;
short x;
short z;

void main_init (void){
	hw_init();
	led_init();
	enableIRQ();
	sys_time_init();
	downlink_init();
	micromag_init();

	IO0DIR |= 1 << 28;
	IO0SET |= 1 << 28;

	/*wait for initialize all periphery*/
	sys_time_wait(500000);
	led_on(LED_GREEN);
}
int i=1;
void main_periodic(void){
	sys_time_periodic_init();
	while(sys_time_periodic()!=1);
	while(1){

		while(sys_time_periodic()!=1);
		led_toggle(LED_RED);
//		MmSendReq(i);
//		if (i==1){
//			x = new_val;
//			if(x>10000){
//				x |= 1 <<15;
//			}
//		}
//		if (i==2){
//			y = new_val;
//		}
//		if (i == 3){
//			z = new_val;
//			i=0;
//		}
//		i++;
////		float angle = atan2(y,x);
////		comm_send_int(angle*100000);
//		if (i == 3){
//		}
//		comm_send_string("x: ");
//		comm_send_int((int)x);
//		comm_send_string(" y: ");
//		comm_send_int((int)y);
//		comm_send_string(" z: ");
//		comm_send_int((int)z);
//		comm_send_string("\r\n");

		if (downlink_char_available()){
			unsigned char read=downlink_get_byte();
			if(read=='0'){
				MmSendReq(0);
			}
			if(read=='1'){
				MmSendReq(1);
			}
			if(read=='2'){
				MmSendReq(2);
			}
			if(read=='3'){
				MmSendReq(3);
			}
			if(read=='p'){
				comm_send_int((int)new_val);
				comm_send_string("\r\n");
			}
			if(read=='s'){
				comm_send_string("MmSet: Reset Pin high\r\n");
				MmSet();
			}
			if(read=='r'){
				comm_send_string("MmReset: Reset Pin low\r\n");
				MmReset();
			}
			if(read=='g'){
				MmReadRes();
			}
			if(read=='m'){
				MmSendMOT();
			}
			if(read=='k'){
				comm_send_string("SlaveSelect: SS Pin low\r\n");
				MmSelect();
			}
			if(read=='l'){
				comm_send_string("SlaveSelect: SS Pin high\r\n");
				MmUnselect();
			}
		}
	}
}

int main(void) {
	main_init();
	main_periodic();
	return 0;
}
