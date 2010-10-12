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
* @file Driver for SCP1000 digital pressure sensor
*   @author Martin Rutschmann <pixhawk@student.ethz.ch>
*
*/

#ifndef SCP1000_H_
#define SCP1000_H_


#define SCP1000_STA_SEND_CONF       0
#define SCP1000_STA_READ_DATA       1

/* SS on P0.2 */
#define SCP_SS_IODIR IO0DIR
#define SCP_SS_IOSET IO0SET
#define SCP_SS_IOCLR IO0CLR
#define SCP_SS_PIN   4

/* DRDY on P0.30 ( EINT1 ) */
#define SCP_DRDY_PINSEL 	PINSEL1
#define SCP_DRDY_PINSEL_BIT	28
#define SCP_DRDY_PINSEL_VAL	2
#define SCP_DRDY_EINT		3
#define SCP_DRDY_VIC_IT		VIC_EINT3

/* SCP1000 measurement methods */
#define SCP_MEAS_HIGH_SPEED 0x09
#define SCP_MEAS_HIGH_RES   0x0A
#define SCP_MEAS_TRIG	    0x0C
#define SCP_MEAS_STOP		0x00

#define SCP_MEAS_MODE 		SCP_MEAS_HIGH_SPEED

void scp1000_init(void);
void scp1000_send_config(void);
unsigned int scp1000_get_value(void);

#endif /* SCP1000_H_ */
