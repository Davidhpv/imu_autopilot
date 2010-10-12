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
*   @brief Driver for PPM sum signal input
*
*   @author Martin Rutschmann <pixhawk@student.ethz.ch>
*
*/

#include "ppm.h"
#include "LPC21xx.h"
#include "sys_time.h"
#include "conf.h"

int ppm_valid;

int ppm_pulses[PPM_NB_CHANNEL];

/**
 * @return 1 is there is a valid PPM signal, 0 else
 */
int ppm_is_valid(void)
{
	return ppm_valid;
}

/**
 *
 * @param nr the channel id, in the range of 1-9
 * @return microseconds this channel had as duty cycle time
 */
int ppm_get_channel(unsigned int nr)
{
	if(nr<1 || nr>PPM_NB_CHANNEL){
		return -1;
	}
	else{
		return SYS_USEC_OF_TICS(ppm_pulses[nr-1]);
	}
}


void ppm_init(void)
{
	/* select pin for capture */
	PPM_CAPTURE_PINSEL |= PPM_CAPTURE_PINSEL_VAL << PPM_CAPTURE_PINSEL_BIT;
	/* enable capture 0.2 on falling edge + trigger interrupt */
	PPM_CAPTURE_CONTROL_REGISTER = PPM_CAPTURE_CONTROL_REGISTER_VALUE;
	int i;
	for(i=0;i<PPM_NB_CHANNEL;i++){
		ppm_pulses[i]=SYS_TICS_OF_USEC(1500);
	}
	ppm_pulses[2]=SYS_TICS_OF_USEC(1000);
	ppm_valid = 0;
}

void PPM_ISR()
{
   	static unsigned int state = PPM_NB_CHANNEL;
   	static unsigned int last;

    unsigned int now = PPM_CAPTURE_CAPTURE_REGISTER;
    unsigned int length = now - last;
    last = now;

    if (state == PPM_NB_CHANNEL) {
      	if (length > SYS_TICS_OF_USEC(PPM_SYNC_MIN_LEN) && length < SYS_TICS_OF_USEC(PPM_SYNC_MAX_LEN)) {
			state = 0;
      	}
      	else{
  			ppm_valid = 0;
      	}
    }
    else {
      	if (length > SYS_TICS_OF_USEC(PPM_DATA_MIN_LEN) && length < SYS_TICS_OF_USEC(PPM_DATA_MAX_LEN)) {
      		ppm_pulses[state] = (int)length;
			state++;
			if (state == PPM_NB_CHANNEL) {
	  			ppm_valid = 1;
			}
    	}
      	else{
			state = PPM_NB_CHANNEL;
  			ppm_valid = 0;
      	}
    }
}
