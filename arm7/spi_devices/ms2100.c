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
* @file Driver for MS2100 magnetometer
*   @author Martin Rutschmann <pixhawk@student.ethz.ch>
*
*/

#include "spi.h"
#include "ms2100.h"
#include "armVIC.h"
#include "conf.h"
#include "sys_time.h"

#include "comm.h"
#include "led.h"
#include "inttypes.h"
#include <stdio.h>

//Slaveselect pin on cam-sw on twog board P1.22
#define MS2100_SS_PIN   22
#define MS2100_SS_IODIR IO0DIR
#define MS2100_SS_IOSET IO0SET
#define MS2100_SS_IOCLR IO0CLR

//Magnet reset pin////////////////////////////////////////////////////////
//Magnet reset pin on ADC3 on twog board P0.04
#define MS2100_RESET_PIN   29
#define MS2100_RESET_IODIR IO0DIR
#define MS2100_RESET_IOSET IO0SET
#define MS2100_RESET_IOCLR IO0CLR

//DATAREADY PIN///////////////////////////////////////////////////////////
//Dataready pin on P0.30/EINT3
#define MS2100_DRDY_PINSEL PINSEL1
#define MS2100_DRDY_PINSEL_BIT 28
#define MS2100_DRDY_PINSEL_VAL 2
#define MS2100_DRDY_EINT 3
#define MS2100_DRDY_VIC_IT VIC_EINT3

static void EXTINT_ISR(void) __attribute__((naked));

/**
 * This function sets the MS2100 pin to low. To reset the sensor you have to
 * first run ms2100_set and then ms2100_reset.
 */
static void ms2100_reset(void);

/**
 * This function sets the MS2100 pin to high. To reset the sensor you have to
 * first run ms2100_set and then ms2100_reset.
 */
static void ms2100_set(void);

/**
 * This function selects the ms2100. If you want to read out a result, then you
 * have to select the ms2100 with this function.
 */
static void ms2100_select(void);

/**
 * This function selects the ms2100 and reset the device. If you want to send
 * a request, you have to choose this select function.
 */
static void ms2100_select_and_reset(void);

/**
 * After a spi transfer is finished, the slave select pin of the ms2100 has to go to high.
 */
static void ms2100_unselect(void);

/**
 * spi receive timeout interrupt function for reading.
 */
static void ms2100_on_spi_int_read(void);

/**
 * spi receive timeout interrupt function for a request.
 */
static void ms2100_on_spi_int_req(void);

void ms2100_do_before_spi_transmit(void);


uint8_t ms2100_status;
uint8_t ms2100_current_axis;
uint8_t ms2100_sampling_nr;
int16_t ms2100_values[3][MS2100_SAMPLING_TIMES];

spi_package package_read;
spi_package package_req;


static void ms2100_reset(void) {
	MS2100_RESET_IOCLR |= 1 << MS2100_RESET_PIN;//TO LOW
}


static void ms2100_set(void) {
	MS2100_RESET_IOSET |= 1 << MS2100_RESET_PIN;//TO HIGH
}


static void ms2100_select_and_reset(void){
	MS2100_SS_IOCLR |= 1 << MS2100_SS_PIN;
	ms2100_set();
	ms2100_reset();
}


static void ms2100_select(void){
	MS2100_SS_IOCLR|=1<<MS2100_SS_PIN;
}


static void ms2100_unselect(void){
	MS2100_SS_IOSET|=1<<MS2100_SS_PIN;
}


void ms2100_init(void) {
	ms2100_status=MS2100_IDLE;
	for (int i = 0; i < 3; i++) {
		for (int j=0; j<MS2100_SAMPLING_TIMES; j++){
			ms2100_values[i][j]=0;
		}
	}

	/* configure SS pin */
	MS2100_SS_IODIR|=1<<MS2100_SS_PIN; /* pin is output  */
	ms2100_unselect(); /* pin idles high */

	/* configure RESET pin */
	MS2100_RESET_IODIR|=1<<MS2100_RESET_PIN; /* pin is output  */
	ms2100_reset(); /* pin idles low  */

	/* configure DRDY pin */
	/* connected pin to EXINT */
	MS2100_DRDY_PINSEL |= MS2100_DRDY_PINSEL_VAL << MS2100_DRDY_PINSEL_BIT;
	EXTMODE|= 1 << MS2100_DRDY_EINT; /* EINT is edge trigered */
	EXTPOLAR|= 1 << MS2100_DRDY_EINT; /* EINT is trigered on rising edge */
	EXTINT|=  1 << MS2100_DRDY_EINT; /* clear pending EINT */

	/* initialize interrupt vector */
	VICIntSelect &= ~VIC_BIT(MS2100_DRDY_VIC_IT); /* select EINT as IRQ source */
	VICIntEnable = VIC_BIT(MS2100_DRDY_VIC_IT); /* enable it                 */
	_VIC_CNTL(MAG_DRDY_VIC_SLOT) = VIC_ENABLE | MS2100_DRDY_VIC_IT;
	_VIC_ADDR(MAG_DRDY_VIC_SLOT) = (unsigned int) EXTINT_ISR; // address of the ISR


	package_read.bit_mode = SPI_8_BIT_MODE;
	/* trigger 2 bytes read */
	package_read.data[0] = 0;
	package_read.data[1] = 0;
	package_read.length = 2;
	package_read.slave_select = &ms2100_select;
	package_read.slave_unselect = &ms2100_unselect;
	package_read.spi_interrupt_handler = &ms2100_on_spi_int_read;

	package_req.bit_mode = SPI_8_BIT_MODE;
	package_req.length = 1;
	package_req.slave_select = &ms2100_select_and_reset;
	package_req.slave_unselect = &ms2100_unselect;
	package_req.spi_interrupt_handler = &ms2100_on_spi_int_req;
}


int ms2100_get_status(void){
	return ms2100_status;
}


void ms2100_send_request(uint8_t axis) {
	if(ms2100_status!=MS2100_IDLE){
		return;//if an other measurement is in process return
	}
	ms2100_status = MS2100_SENDING_REQ; //set status
	ms2100_current_axis = axis;
	ms2100_sampling_nr = 0;
	unsigned char cmd = 2 << 4 | (ms2100_current_axis+1) << 0;
	package_req.data[0] = cmd;
	spi_transmit(&package_req);
}


short ms2100_get_value(int axis){
	return (short)(0.0373*ms2100_values[axis][0] + 0.0639*ms2100_values[axis][1] + 0.1005*ms2100_values[axis][2] + 0.1326*ms2100_values[axis][3] + 0.1514*ms2100_values[axis][4] + 0.1514*ms2100_values[axis][5]+ 0.1326*ms2100_values[axis][6] + 0.1005*ms2100_values[axis][7] + 0.0639*ms2100_values[axis][8] + 0.0373*ms2100_values[axis][9]);
//	return (short)ms2100_values[axis][0];
}


static void ms2100_on_spi_int_read(void)
{
	//uint8_t buffer[200];	// string buffer for debug messages
	//sprintf((char *)buffer, "ms2100 read done\n");
	//message_send_debug(COMM_0, buffer);

	short new_val;
	new_val = SSPDR << 8;
	new_val += SSPDR;

	if((new_val) > 300 || new_val < -300){
		//there was a outliner we take the stay at the last measurement
	}
	else{
		ms2100_values[ms2100_current_axis][ms2100_sampling_nr] = new_val;
	}

	ms2100_sampling_nr++;

	if(ms2100_sampling_nr>=MS2100_SAMPLING_TIMES){
		ms2100_status = MS2100_IDLE;
	}
	else{
		ms2100_status = MS2100_SENDING_REQ;
		spi_transmit(&package_req);
	}

	return;
}


static void ms2100_on_spi_int_req(void){
	/* read dummy control byte reply */
	//uint8_t buffer[200];	// string buffer for debug messages
	//sprintf((char *)buffer, "ms2100 request done\n");
	//message_send_debug(COMM_0, buffer);

	unsigned char foo __attribute__((unused)) = SSPDR;
	ms2100_status = MS2100_WAITING_EOC;

	return;
}

void ms2100_do_before_spi_transmit(void) {

	MS2100_SS_IOCLR|=1<<MS2100_SS_PIN;

	uint32_t start_time_tc;
	start_time_tc = T0TC;

	//uint8_t buffer[200];	// string buffer for debug messages
	//sprintf((char *)buffer, "ms2100 before transmit 1\n");
	//message_send_debug(COMM_0, buffer);


	if (MAX_TC_VAL - start_time_tc >= 2*MS2100_RESET_TIMEOUT_BASE) {

		while(T0TC-start_time_tc < 2*MS2100_RESET_TIMEOUT_BASE) ;
	}
	else {

		while(T0TC<2*MS2100_RESET_TIMEOUT_BASE-1) ;

	}

	ms2100_set();
	if(MAX_TC_VAL - start_time_tc >= 3*MS2100_RESET_TIMEOUT_BASE) {
		while(T0TC-start_time_tc < 3*MS2100_RESET_TIMEOUT_BASE) ;
	}
	else {
		while(T0TC<3*MS2100_RESET_TIMEOUT_BASE-1) ;
	}
	ms2100_reset();
	//uint8_t buffer[200];	// string buffer for debug messages
	//sprintf((char *)buffer, "ms2100 before transmit 2\n");
	//message_send_debug(COMM_0, buffer);

}


static void EXTINT_ISR(void) {
	ISR_ENTRY();

	//uint8_t buffer[200];	// string buffer for debug messages
	//sprintf((char *)buffer, "ms2100 interrupt\n");
	//message_send_debug(COMM_0, buffer);

    ms2100_status = MS2100_READING_RES; //set status
	spi_transmit(&package_read);

	/* clear EINT */
	EXTINT|=  1 << MS2100_DRDY_EINT;

	VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
	ISR_EXIT();
}
