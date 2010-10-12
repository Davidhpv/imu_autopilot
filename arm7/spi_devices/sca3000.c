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
* @file Driver for SCA3000 accelerometer
*   @author Martin Rutschmann <pixhawk@student.ethz.ch>
*
*/


#include "spi.h"
#include "sca3000.h"
#include "armVIC.h"
#include "conf.h"


#ifdef BOARD_PIXHAWK_V100

int sca3000_values[3];

void sca3000_spi_init(void);

static void sca3000_select(void);
static void sca3000_unselect(void);
static void sca3000_on_spi_int(void);

static void sca3000_select(void){
	SCA3000_SS_IOCLR|=1<<SCA3000_SS_PIN;
}
static void sca3000_unselect(void){
	SCA3000_SS_IOSET|=1<<SCA3000_SS_PIN;
}


void sca3000_init(void) {
	/* configure SS pin */
	SCA3000_SS_IODIR|=1<<SCA3000_SS_PIN; /* pin is output  */
	sca3000_unselect(); /* pin idles high */

	for(int i=0; i<3; i++){
		sca3000_values[i]=0;
	}
}



void sca3000_read_res(void) {
    spi_package package;
	package.bit_mode = SPI_8_BIT_MODE;

	/* trigger 2 bytes read */
    unsigned char cmd=0x05<<2;
	package.data[0] = cmd;
	package.data[1] = 0;
	package.data[2] = 0;
	package.data[3] = 0;
	package.data[4] = 0;
	package.data[5] = 0;
	package.data[6] = 0;
	package.length = 7;
	package.slave_select = &sca3000_select;
	package.slave_unselect = &sca3000_unselect;
	package.spi_interrupt_handler = &sca3000_on_spi_int;
	spi_transmit(&package);
}

void sca3000_on_spi_int(void) {

			/* read dummy control byte reply */
			unsigned char foo __attribute__ ((unused)) = SSPDR;
			signed char msb;
			msb = SSPDR;
			unsigned char lsb = SSPDR;
			sca3000_values[SCA3000_Y_AXIS] = msb*32 + lsb/8;

			msb = SSPDR;
			lsb = SSPDR;
			sca3000_values[SCA3000_Z_AXIS] = msb*32 + lsb/8;

			msb = SSPDR;
			lsb = SSPDR;
			sca3000_values[SCA3000_X_AXIS] = msb*32 + lsb/8;

			return;
}



int sca3000_get_value(int axis){
	return sca3000_values[axis];
}

#endif
