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
 *   @brief Microchip 24FC256 EEPROM driver
 *
 *   This file contains the device driver for the Microchip 24FC256 EEPROM.
 *   @author Christian Dobler <mavteam@student.ethz.ch>
 *
 */

#include "inttypes.h"
#include "eeprom.h"
#include "conf.h"
#include "i2c.h"
#include "debug.h"

#include "comm.h"
#include "led.h"
#include <stdio.h>

#if (FEATURE_EEPROM==FEATURE_EEPROM_ENABLED)

uint8_t data_buffer[MAX_I2C_PACKAGE_SIZE];
uint8_t data_ready = 0;

/**
 * @brief Temporarily saves read results within the EEPROM driver
 *
 * This function is invoked by the I2C subsystem upon completion of the EEPROM read.
 * It stores the read data temporarily within the EEPROM driver. The user application
 * can extract this data from the driver by calling "eeprom_read_data()".
 *
 * @param package 	I2C package as described in i2c.h
 */
static void eeprom_save_data_after_read(i2c_package *package);


void eeprom_write(uint16_t address, uint8_t length, uint8_t * data)
{
	uint8_t copy_counter;
	
	// check that byte array is not longer than 64 bytes (max. page write length)
	if(length>64) {
		//uint8_t buffer[200];	// string buffer for debug messages
		//sprintf((char *)buffer, "EEPROM error: Maximum page write size is 64 bytes\n");
		//message_send_debug(COMM_0, buffer);
		return;
	}

	// set up I2C package to be passed to the I2C subsystem
	i2c_package package;
	package.data[1] = (uint8_t)address;					// LSB of write address within the EEPROM
	package.data[0] = (uint8_t)(address>>8);			// MSB of write address within the EEPROM
	package.length = length+2;							// data bytes + 2 address bytes
	package.direction = I2C_WRITE;						// I2C write operation
	package.slave_address = EEPROM_I2C_SLAVE_ADDRESS;	// I2C slave address of EEPROM
	package.bus_number = EEPROM_I2C_BUS_NUMBER;			// number of the I2C bus, that the EEPROM is connected to
	package.write_read = 0;								// no repeated start condition
	package.i2c_done_handler = NULL;					// nothing to be done at I2C completion

	// copy user data to I2C package
	for(copy_counter=0; copy_counter<length; copy_counter++)
	{
		package.data[copy_counter+2] = data[copy_counter];
	}

	i2c_op(&package);	// start I2C write operation
}


void eeprom_start_read(uint16_t address, uint8_t length)
{
	i2c_package package_write, package_read;
	
	// check that byte array is not longer than MAX_I2C_PACKAGE_SIZE bytes (max. transfer size of I2C subsystem)
	if(length>MAX_I2C_PACKAGE_SIZE) {
		//uint8_t buffer[200];	// string buffer for debug messages
		//sprintf((char *)buffer, "EEPROM error: too large I2C package size specified\n");
		//message_send_debug(COMM_0, buffer);
		return;
	}

	// set up I2C package for the read command to be passed to the I2C subsystem
	package_write.data[1] = (uint8_t)address;						// LSB of read address within the EEPROM
	package_write.data[0] = (uint8_t)(address>>8);					// MSB of read address within the EEPROM
	package_write.length = 2;										// 2 address bytes
	package_write.direction = I2C_WRITE;							// I2C write operation
	package_write.slave_address = EEPROM_I2C_SLAVE_ADDRESS; 		// I2C slave address of EEPROM
	package_write.bus_number = EEPROM_I2C_BUS_NUMBER;				// number of the I2C bus, that the EEPROM is connected to
	package_write.write_read = 1;									// repeated start condition at completion
	package_write.i2c_done_handler = NULL;							// nothing to be done at I2C completion

	// set up I2C package for the data read-out to be passed to the I2C subsystem
	package_read.length = length;									// number of bytes to be read
	package_read.direction = I2C_READ;								// I2C read operation
	package_read.slave_address = EEPROM_I2C_SLAVE_ADDRESS;			// I2C slave address of EEPROM
	package_read.bus_number = EEPROM_I2C_BUS_NUMBER;				// number of the I2C bus, that the EEPROM is connected to
	package_read.write_read = 1;									// repeated start condition at start
	package_read.i2c_done_handler = (void*)&eeprom_save_data_after_read;	// eeprom_save_data_after_read() is called at I2C completion

	i2c_write_read(&package_write, &package_read);  // start data read operation

}

void eeprom_save_data_after_read(i2c_package *package)
{
	uint8_t copy_counter;

	// copy data from the I2C package passed by the I2C subsystem to the temporary data buffer within this driver
	for(copy_counter=0; copy_counter<package->length; copy_counter++)
	{
		data_buffer[copy_counter] = package->data[copy_counter];
	}

	// set data ready to the number of valid bytes in the data buffer
	data_ready = package->length;
}

void eeprom_read_data(uint8_t * data)
{
	uint8_t copy_counter;

	// wait for data to be read from the EEPROM
	while(!data_ready) ;

	// copy the read data to the byte attay "data" specified by the user application
	for(copy_counter=0; copy_counter<data_ready; copy_counter++)
		{
			data[copy_counter] = data_buffer[copy_counter];
		}

	// reset data_ready
	data_ready = 0;
}

static int8_t eeprom_available=-1;

void eeprom_check_start()
{
//	TODO change this to use just one package with handler


	//start reading address 0 from eeprom. this will fail if there is none. Then eeprom_check_handler is checking error code from i2c
	i2c_package package_write, package_read;

	// set up I2C package for the read command to be passed to the I2C subsystem
	package_write.data[1] = (uint8_t)0;						// LSB of read address within the EEPROM
	package_write.data[0] = (uint8_t)(0>>8);					// MSB of read address within the EEPROM
	package_write.length = 2;										// 2 address bytes
	package_write.direction = I2C_WRITE;							// I2C write operation
	package_write.slave_address = EEPROM_I2C_SLAVE_ADDRESS; 		// I2C slave address of EEPROM
	package_write.bus_number = EEPROM_I2C_BUS_NUMBER;				// number of the I2C bus, that the EEPROM is connected to
	package_write.write_read = 1;									// repeated start condition at completion
	package_write.i2c_done_handler = NULL;							// nothing to be done at I2C completion

	// set up I2C package for the data read-out to be passed to the I2C subsystem
	package_read.length = 1;									// number of bytes to be read
	package_read.direction = I2C_READ;								// I2C read operation
	package_read.slave_address = EEPROM_I2C_SLAVE_ADDRESS;			// I2C slave address of EEPROM
	package_read.bus_number = EEPROM_I2C_BUS_NUMBER;				// number of the I2C bus, that the EEPROM is connected to
	package_read.write_read = 1;									// repeated start condition at start
	package_read.i2c_done_handler = (void*)&eeprom_check_handler;	// eeprom_save_data_after_read() is called at I2C completion

	i2c_write_read(&package_write, &package_read);  // start data read operation
}

void eeprom_check_handler(i2c_package *package)
{
	debug_message_buffer("YAAAAY!!! eeprom_check_handler executed");
	if(package->i2c_error_code== I2C_CODE_ERROR)
	{
		eeprom_available=0;
	}
	else
	{
		eeprom_available=1;
	}
}


int8_t eeprom_check_ok(){
//	while(eeprom_available==-1){
////		wait
//	}
	if(eeprom_available==-1)
	{
		debug_message_buffer("eeprom_check is still -1 assuming 0");
		return 0;
	}
	return eeprom_available;
}
#endif
