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
 * @file i2c.h
 *   @brief I2C interface driver
 *
 *   This file contains the I2C driver.
 *   @author Christian Dobler <mavteam@student.ethz.ch>
 *
 */
 
#ifndef I2C_H_
#define I2C_H_

#include "inttypes.h"
#include "conf.h"

#if(FEATURE_I2C == FEATURE_I2C_ENABLED	)

#define I2C_WRITE				0		///< definition of the R/W bit for a I2C write operation
#define I2C_READ				1		///< definition of the R/W bit for a I2C read operation

#define I2C_CODE_NOT_KNOWN		0
#define I2C_CODE_OK				1
#define I2C_CODE_ERROR			2

/**
 * @struct i2c_package This structure holds all the data necessary for a complete I2C operation.
 */
typedef struct {

	/*! pointer to data array*/
	uint8_t data[MAX_I2C_PACKAGE_SIZE];
	
	/*! the number of bytes in the data array */
	uint8_t length;
	
	/*! direction of I2C Operation: write: 0, read: 1 */
	uint8_t direction;  
	
	/*! address of slave device to communicate with */
	uint8_t slave_address;

	/*! number of the I2C bus the addresses slave device is attached to: 0 or 1 */
	uint8_t bus_number;
	
	/*! set this variable to 1, if a repeated start should be made between a write
	 * package and the following read package */
	uint8_t write_read;

	/*! function to be defined by the specific device driver to handle the received 
	 *  data at the end of a I2C read operation */
	void (*i2c_done_handler)(void * package);

	/*! I2C error code */
	uint8_t i2c_error_code;

	/*! I2C error Counter number of trys */
	uint8_t i2c_error_counter;


} i2c_package;


/**
 * @brief Initializes the I2C interfaces
 *
 * This function initializes the I2C subsystem. It has to be executed before any 
 * I2C device driver.
 *
 */
void i2c_init(void);

/**
 * @brief Write and Read function for reading specific addresses in I2C devices
 *
 * This function first issues a I2C write package followed by an I2C read package
 * with a repeated start condition in between. Use this function to read a specific
 * address from an I2C device by first transmitting the read address in the write
 * package and then really reading the data with the read package.
 *
 * @param package_write 	I2C write package (direction variable set to 0)
 * @param package_read		I2C read package (direction variable set to 1)
 */
void i2c_write_read(i2c_package * package_write, i2c_package * package_read);


/**
 * @brief Function to start an I2C operation
 *
 * This function should by called by I2C device drivers to start an I2C operation.
 * The details about the operation to be performed is passed to the I2C interface
 * driver in the I2C "package". See the definition of the structure i2c_package
 * for a detailed description.
 *
 * @param package 	I2C package as defined in this file
 */
void i2c_op(i2c_package * package);

#endif

#endif
