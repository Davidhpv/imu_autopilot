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
 *   @brief SCA3100 linear accelerometer device driver
 *
 *   This file contains the device driver for the VTI SCA3100 linear accelerometer.
 *   @author Christian Dobler <mavteam@student.ethz.ch>
 *
 */

#include "spi.h"
#include "sca3100.h"
#include "armVIC.h"
#include "conf.h"
#include "comm.h"
#include "led.h"
#include "inttypes.h"
#include <stdio.h>




#if(FEATURE_ACC==FEATURE_ACC_3100)

int sca3100_values[3];						// temporary storage for measured acceleration values

void sca3100_spi_init(void);

static void sca3100_select(void);
static void sca3100_unselect(void);
static void sca3100_on_spi_int(void);
static void sca3100_on_spi_read_reg(void);
static void sca3100_on_spi_write(void);

static void sca3100_select(void){			// pull SCA3100 chip select low
	SCA3100_SS_IOCLR|=1<<SCA3100_SS_PIN;
}
static void sca3100_unselect(void){			// pull SCA3100 chip select high
	SCA3100_SS_IOSET|=1<<SCA3100_SS_PIN;
}


void sca3100_init(void) {
	/* configure SS pin */
	SCA3100_SS_IODIR|=1<<SCA3100_SS_PIN; 	/* pin is output  */
	sca3100_unselect(); 					/* pin idles high */
	
	//spi_package package;
	
	/* set PORST bit to 0 */
	/*package.data[0] = 0x07;
	package.data[1] = 0x00;
	package.length = 2;
	package.slave_select = &sca3100_select;
	package.slave_unselect = &sca3100_unselect;
	package.spi_interrupt_handler = &sca3100_on_spi_int;
	spi_transmit(&package);*/
	
	//led_on(LED_GREEN);

	// set all measurement results to 0
	for(int i=0; i<3; i++){
		sca3100_values[i]=0;
	}
	sca3100_reset_porst_bit();
	sca3100_read_int_status();
}


// start a measurement read operation
void sca3100_read_res(void) {
	
    spi_package package;
	package.bit_mode = SPI_8_BIT_MODE;

	/* trigger 2 bytes read */
    unsigned char cmd = 0x15;		// command byte: start with reading axis, address counter increments automatically
	package.data[0] = cmd;			// send command byte first
	package.data[1] = 0;
	package.data[2] = 0;
	package.data[3] = 0;
	package.data[4] = 0;
	package.data[5] = 0;
	package.data[6] = 0;
	package.length = 7;				// send 1 command byte and read 6 data bytes
	package.slave_select = &sca3100_select;
	package.slave_unselect = &sca3100_unselect;
	package.spi_interrupt_handler = &sca3100_on_spi_int;	// sca3100_on_spi_int() is called at SPI completion
	spi_transmit(&package);			// start SPI operation
}

static void sca3100_on_spi_int(void) {


	/* read dummy control byte reply */
	unsigned char sca3100_status = SSPDR;	// temporarily save received SCA3100 status byte (first byte received)
	//uint8_t buffer[200];
	spi_package package;
	package.bit_mode = SPI_8_BIT_MODE;
	unsigned char cmd;
	unsigned char cmd_val;
	unsigned char val_ok = 1;	// start with val_ok=1, if anything went wrong, val_ok will be set to 0 and new values discarded
	
	if((sca3100_status & 0x7C) || !(sca3100_status & (1<<1))) {
	
		// SPI frame error in previously sent frame was detected
		if(sca3100_status & (1<<6)) {
			//sprintf((char *)buffer, "FRME: SPI frame error (previous frame)\n");
			//message_send_debug(COMM_1, buffer);
		}
		
		// sensor has been reset since the last measurement
		if(sca3100_status & (1<<5)) {
			//sprintf((char *)buffer, "PORST: unexpected sensor reset occurred\n");
			//message_send_debug(COMM_1, buffer);
			val_ok = 0;
			
		}
		
		// one of the device self tests has failed
		if(sca3100_status & (1<<4)) {
			val_ok = 0;
			//led_on(LED_GREEN);
		}
		
		// sensor was in saturation, when measuring the current value -> discard measurement
		if(sca3100_status & (1<<3)) {
			led_on(LED_GREEN);
			//sprintf((char *)buffer, "SAT: sensor saturation occurred\n");
			//message_send_debug(COMM_1, buffer);
			val_ok = 0;
		}
		
		// fixed SCA3100 control bits are received incorrectly
		if((sca3100_status & (1<<2)) || !(sca3100_status & (1<<1))) {
			//sprintf((char *)buffer, "FRME: SPI receive error (current frame)\n");
			//message_send_debug(COMM_1, buffer);
			val_ok = 0;
		}
			
	}

	// if status byte is ok, copy and convert measurement data to mg
	if(val_ok)
	{
		// X axis first:
		unsigned char msb;
		msb = SSPDR;					// read MSB from SPI data register
		unsigned char lsb = SSPDR;		// read LSB from SPI data register
		// Convert 2's complement (Manual page 14, section 3.1.4 Output data conversion)
		// Copy msb and lsb to (msb|lsb)
		uint16_t raw_value = ((uint16_t)msb<<8) | lsb;
		int16_t mg_value;
		// Align bit pattern correctly to 16bit, last two bits not defined
		raw_value = raw_value>>2;
		// convert raw_value in two's complement to mg_value in normal binary format and scale it to mg
		if(raw_value & (1<<13))
		{
			raw_value &= 0x1FFF;
			mg_value = ((int16_t)raw_value-SCA3100_MAX_NEG_VALUE);
		}
		else {
			mg_value = (int16_t)raw_value;
		}
		sca3100_values[SCA3100_X_AXIS] = (int)mg_value; 	// store measurement for X axis

		// same for the Z axis:
		msb = SSPDR;					// read MSB from SPI data register
		lsb = SSPDR;					// read LSB from SPI data register
		raw_value = ((uint16_t)msb<<8) | lsb;
		raw_value = raw_value>>2;
		// convert raw_value in two's complement to mg_value in normal binary format and scale it to mg
		if(raw_value & (1<<13)) {
			raw_value &= 0x1FFF;
			mg_value = ((int16_t)raw_value-SCA3100_MAX_NEG_VALUE);
		}
		else {
			mg_value = (int16_t)raw_value;
		}
		sca3100_values[SCA3100_Z_AXIS] = (int)mg_value;		// store measurement for Z axis

		// same for the Y axis:
		msb = SSPDR;					// read MSB from SPI data register
		lsb = SSPDR;					// read LSB from SPI data register
		raw_value = ((uint16_t)msb<<8) | lsb;
		raw_value = raw_value>>2;
		// convert raw_value in two's complement to mg_value in normal binary format and scale it to mg
		if(raw_value & (1<<13)) {
			raw_value &= 0x1FFF;
			mg_value = ((int16_t)raw_value-SCA3100_MAX_NEG_VALUE);
		}
		else {
			mg_value = (int16_t)raw_value;
		}
		sca3100_values[SCA3100_Y_AXIS] = (int)mg_value;		// store measurement for Y axis
		
	}
	
	// discard received data, if status byte from SCA3100 was not ok
	else {
		int i;
		unsigned char foo __attribute__ ((unused));
		for(i=0; i<6;i++) {
			foo = SSPDR;
		}
	}
	
	// if anything in the status byte was abnormal, do the following
	if(sca3100_status & 0x7C) {
	
		// if the PORST bit was set, issue the write command 0x0700 to clear it (not supposed to happen under normal operation)
		if(sca3100_status & (1<<5)) {
			cmd = 0x07;
			cmd_val = 0x00;
			package.data[0] = cmd;
			package.data[1] = cmd_val;
			package.length = 2;
			package.slave_select = &sca3100_select;
			package.slave_unselect = &sca3100_unselect;
			package.spi_interrupt_handler = &sca3100_on_spi_write;	// sca3100_on_spi_write() is invoked at SPI completion
			spi_transmit(&package);
			led_on(LED_GREEN);
			return;
		}
		
		
		// if the SAT or ST bit was set, read the INT_STATUS register to determine exact cause and clear flags (not supposed to happen under normal operation)
		if((sca3100_status & (1<<4)) || (sca3100_status & (1<<3))) {
			led_on(LED_GREEN);
			cmd = 0x58;							// 0x5800 is the command to read the status register
			cmd_val = 0x00;						// all commands: page 23 in SCA3100 family datasheet
			package.data[0] = cmd;
			package.data[1] = cmd_val;
			package.length = 2;
			package.slave_select = &sca3100_select;
			package.slave_unselect = &sca3100_unselect;
			package.spi_interrupt_handler = &sca3100_on_spi_read_reg;	// sca3100_on_spi_read_reg() is invoked at SPI completion
			spi_transmit(&package);
			return;
		}
			
	}
	
	return;
}


int sca3100_get_value(int axis){
	return sca3100_values[axis];	// return value of specified axis
}

void sca3100_reset_porst_bit(void) {
	spi_package package;
	package.bit_mode = SPI_8_BIT_MODE;
	unsigned char cmd;
	unsigned char cmd_val;
	cmd = 0x07;					// 0x0700 is the command to reset the PORST bit
	cmd_val = 0x00;
	package.data[0] = cmd;
	package.data[1] = cmd_val;
	package.length = 2;
	package.slave_select = &sca3100_select;
	package.slave_unselect = &sca3100_unselect;
	package.spi_interrupt_handler = &sca3100_on_spi_write;	// sca3100_on_spi_write() is invoked at SPI completion
	spi_transmit(&package);
	//led_on(LED_GREEN);
	return;
}

void sca3100_read_int_status(void) {
	spi_package package;
	package.bit_mode = SPI_8_BIT_MODE;
	unsigned char cmd;
	unsigned char cmd_val;
	cmd = 0x58;					// 0x5800 is the command to read the status register
	cmd_val = 0x00;
	package.data[0] = cmd;
	package.data[1] = cmd_val;
	package.length = 2;
	package.slave_select = &sca3100_select;
	package.slave_unselect = &sca3100_unselect;
	package.spi_interrupt_handler = &sca3100_on_spi_read_reg;	// sca3100_on_spi_read_reg() is invoked at SPI completion
	spi_transmit(&package);
	return;
}

static void sca3100_on_spi_write(void) {
	unsigned char foo __attribute__ ((unused)) = SSPDR;	// this was a write operation -> discard any received data (2 bytes)
	foo = SSPDR;
	//led_on(LED_GREEN);
	return;
}

static void sca3100_on_spi_read_reg(void) {		// INT_STATUS register has been read
	//uint8_t buffer[200];
	spi_package package;
	package.bit_mode = SPI_8_BIT_MODE;
	unsigned char cmd;
	unsigned char cmd_val;
	unsigned char foo __attribute__ ((unused)) = SSPDR;	// discard first byte
	unsigned char int_status = SSPDR;					// INT_STATUS register content is in the second byte (this byte)
	// if STS has failed
	if(int_status & (1<<5)) {
		//sprintf((char *)buffer, "STS: Start-up self-test failed\n");
		//message_send_debug(COMM_1, buffer);
	}
	// if STC has failed
	if(int_status & (1<<4)) {
		//sprintf((char *)buffer, "STC: Continuous self-test failed\n");
		//message_send_debug(COMM_1, buffer);
	}
	// if not SAT & not STS & not STC, then memory self test has failed -> do another memory self-test
	if(!((int_status & (1<<6)) || (int_status & (1<<4)) || (int_status & (1<<5)))) {
		//sprintf((char *)buffer, "ST: Memory self test-failed\n");
		//message_send_debug(COMM_1, buffer);
		cmd = 0x07;				// 0x0704 is the command to start a new memory self test
		cmd_val = 0x04;
		package.data[0] = cmd;
		package.data[1] = cmd_val;
		package.length = 2;
		package.slave_select = &sca3100_select;
		package.slave_unselect = &sca3100_unselect;
		package.spi_interrupt_handler = &sca3100_on_spi_write;	// sca3100_on_spi_write() is invoked at SPI completion
		spi_transmit(&package);
	}
	led_off(LED_GREEN);
	return;
}

#endif

