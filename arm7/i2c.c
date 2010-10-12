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
 *   @brief I2C interface driver
 *
 *   This file contains the I2C driver.
 *   @author Christian Dobler <mavteam@student.ethz.ch>
 *
 */

#include "i2c.h"
#include "armVIC.h"
#include "LPC21xx.h"
#include "conf.h"

#include "comm.h"
#include "led.h"
#include "inttypes.h"
#include <stdio.h>
#include "global_data.h"

#include "debug.h"

#if(FEATURE_I2C == FEATURE_I2C_ENABLED	)

static void I2C0_ISR(void) __attribute__((naked));
// interrupt handler for I2C Bus 0
static void I2C1_ISR(void) __attribute__((naked));
// interrupt handler for I2C Bus 1

/* function to start an I2C operation on I2C0 (not to be called by device drivers) */
void i2c0_op(i2c_package * package);
/* function to start an I2C operation on I2C1 (not to be called by device drivers) */
void i2c1_op(i2c_package * package);

i2c_package package_buffer0[I2C_PACKAGE_BUFFER_SIZE]; // package buffer for I2C0
i2c_package package_buffer1[I2C_PACKAGE_BUFFER_SIZE]; // package buffer for I2C1

uint8_t i2c_package_buffer0_insert_idx, i2c_package_buffer0_current_idx; // insertion and processing pointers for I2C0
uint8_t i2c_package_buffer1_insert_idx, i2c_package_buffer1_current_idx; // insertion and processing pointers for I2C1
uint8_t current_data_byte_i2c0, current_data_byte_i2c1; // data byte counters
uint8_t i2c0_busy, i2c1_busy; // I2C resource busy flags

uint16_t error_counter0, error_counter1; // counter variables to count number of errors per package


void i2c_init(void)
{
	/* setup pins for I2C0 (SCL0, SDA0) and I2C1 (SCL1, SDA1) */
	PINSEL0 |= I2C0_PINSEL0_SCL | I2C0_PINSEL0_SDA | I2C1_PINSEL0_SCL
			| I2C1_PINSEL0_SDA;

	/* setup I2C0 */
	I2C0ADR = LPC_I2C_ADR;
	I2C0SCLH = I2C0SCLH_VAL;
	I2C0SCLL = I2C0SCLL_VAL;
	I2C0CONSET = I2C0CONSET_VAL;

	/* initialize I2C0 interrupt vector */
	VICIntSelect &= ~VIC_BIT( VIC_I2C0 ); /* I2C0 selected as IRQ 	*/
	VICIntEnable = VIC_BIT( VIC_I2C0 ); /* enable it            	*/
	_VIC_CNTL(I2C0_VIC_SLOT) = VIC_ENABLE | VIC_I2C0;
	_VIC_ADDR(I2C0_VIC_SLOT) = (unsigned int) I2C0_ISR; /* address of the I2C0 ISR	*/

	/* setup I2C1 */
	I2C1ADR = LPC_I2C_ADR;
	I2C1SCLH = I2C1SCLH_VAL;
	I2C1SCLL = I2C1SCLL_VAL;
	I2C1CONSET = I2C1CONSET_VAL;

	/* initialize I2C1 interrupt vector */
	VICIntSelect &= ~VIC_BIT( VIC_I2C1 ); /* I2C1 selected as IRQ 	*/
	VICIntEnable = VIC_BIT( VIC_I2C1 ); /* enable it            	*/
	_VIC_CNTL(I2C_VIC_SLOT) = VIC_ENABLE | VIC_I2C1;
	_VIC_ADDR(I2C_VIC_SLOT) = (unsigned int) I2C1_ISR; /* address of the I2C1 ISR	*/

	// insertion and processing pointers for I2C0
	i2c_package_buffer0_insert_idx = 0;
	i2c_package_buffer0_current_idx = 0;
	i2c_package_buffer1_insert_idx = 0;
	i2c_package_buffer1_current_idx = 0;

	// status variables for I2C Bus 0 and 1
	i2c0_busy = 0;
	i2c1_busy = 0;

}

void i2c_write_read(i2c_package * package_write, i2c_package * package_read)
{

	package_write->write_read = 1;
	package_read->write_read = 1;

	i2c_op(package_write); // I2C wirite operation first
	i2c_op(package_read); // just followed by I2C read operation

	return;
}

void i2c_op(i2c_package * package)
{

	//uint8_t buffer[200];	// string buffer for debug messages
	switch (package->bus_number)
	{ // decide wether device is on bus 0 or 1
	case 0:
		i2c0_op(package);
		break;
	case 1:
		i2c1_op(package);
		break;
	default:
		//sprintf((char *)buffer, "I2C error: non existing bus number specified\n");
		//message_send_debug(COMM_1, buffer);
		break;
	}

	return;
}

/* the same for I2C1 */
uint32_t i2c1_reject_count = 0;
/* I2C1 bus operation starter function */
void i2c1_op(i2c_package * package)
{
	//setup error codes
	//	package.i2c_error_code=I2C_CODE_NOT_KNOWN;
	//	package.i2c_error_counter=0;
	//TODO why doesn't this work?

	// If Buffer is full we can't send a package
	if (i2c_package_buffer1_current_idx - i2c_package_buffer1_insert_idx == 1
			|| (i2c_package_buffer1_current_idx == 0
					&& i2c_package_buffer1_insert_idx
							== (I2C_PACKAGE_BUFFER_SIZE - 1)))
	{

		//TODO Change function to return an error code and adapt all calls to check if they where successful. 20100428 Laurens

// DIRTY HACK: a motorcontroller is pulling SDA down we will restart

		I2C1CONSET = 1 << STO; // generate I2C stop condition on the bus without restart
		I2C1CONSET = 1 << STA;
//		I2C1CONCLR = 1 << SIC; // clear I2C interrupt flag


//		debug_message_buffer_sprintf(
//			"i2c I2C1STAT is: %i.",
//			I2C1STAT);

		debug_message_buffer_sprintf("*****Restarted I2C1*** I2C1STAT=%i",I2C1STAT);
//END HACK

		if (i2c1_reject_count++ % 256 == 0)
		{
			debug_message_buffer_sprintf(
					"i2c1 buffer full. Rejected package. Total: %i",
					i2c1_reject_count);


		}
		return;
	}

	package_buffer1[i2c_package_buffer1_insert_idx] = *package; // add new i2c package to package buffer 1
	i2c_package_buffer1_insert_idx = (i2c_package_buffer1_insert_idx + 1)
			% I2C_PACKAGE_BUFFER_SIZE; // increment package insertion pointer

	if (!i2c1_busy)
	{ // check whether I2C1 is busy: if busy, data will I2C operation will be executed later, but program execution can continue
		i2c1_busy = 1; // process locks I2C resource
		I2C1CONSET = 1 << STA; // I2C start condition is generated on the bus -> interrupt is generated when ready; next state code: 0x08
		error_counter1 = 0;
	}

	return;
}

uint32_t i2c1_permanent_error_count = 0;
volatile int i2c1stat_prior_state = 0;
/* Interrupt handler for I2C1 interrupt */
static void I2C1_ISR(void)
{

	//sprintf((char*)buffer, "h ");
	//message_send_debug(COMM_1, buffer);
	//uint8_t buffer[200];	// string buffer for debug messages
	ISR_ENTRY();

	if (error_counter1 > I2C_PERMANENT_ERROR_LIMIT)
	{
		//uint8_t buffer[50];
		//sprintf(buffer, "i2c error limit %d \n", 1);
		//  sprintf((char*)buffer, package_buffer1[i2c_package_buffer1_current_idx].slave_address) ;
		//message_send_debug(COMM_1, buffer);
		//		debug_message_buffer("i2c error limit reached");
		debug_message_buffer_sprintf("i2c1 error limit reached. Dest: %i",
				package_buffer1[i2c_package_buffer1_current_idx].slave_address);

		if (i2c1_permanent_error_count++ % 256 == 0)
		{
			debug_message_buffer_sprintf(
					"i2c1 error limit reached. Total: %i errors.",
					i2c1_permanent_error_count);

		}

		package_buffer1[i2c_package_buffer1_current_idx].i2c_error_code
				= I2C_CODE_ERROR;//set error code to error
		//TODO set this to ok if everzthing was ok

		if (package_buffer1[i2c_package_buffer1_current_idx].i2c_done_handler
				!= NULL)
		{ // check if there is a package handler registered
			package_buffer1[i2c_package_buffer1_current_idx].i2c_done_handler(
					&(package_buffer1[i2c_package_buffer1_current_idx])); // call package handler
		}

		i2c_package_buffer1_insert_idx = (i2c_package_buffer1_insert_idx + 1)
				% I2C_PACKAGE_BUFFER_SIZE; // increment package insertion pointer
		error_counter1 = 0;
		if (i2c_package_buffer1_current_idx == i2c_package_buffer1_insert_idx)
		{ // no unhandled packages
			I2C1CONCLR = 1 << STAC;
			I2C1CONSET = 1 << STO; // generate I2C stop condition on the bus without restart
			I2C1CONCLR = 1 << SIC; // clear I2C interrupt flag
			i2c1_busy = 0; // release I2C resource
		}
		else
		{ // unhandled packages in package buffer
			if (!(package_buffer1[i2c_package_buffer1_current_idx].write_read
					== 1))
			{
				I2C1CONSET = 1 << STO; // generate stop condition on the bus if no repeated start is necessary
			}
			I2C1CONSET = 1 << STA; // generate "repeated start" condition on the bus -> next state: 0x10
			I2C1CONCLR = 1 << SIC; // clear I2C interrupt flag
		}
	}
	if (i2c1_busy == 1)
	{ //Return if Package error
		//message_debug_send(MAVLINK_COMM_1, 30, I2C1STAT);
		switch (I2C1STAT)
		{ // check current I2C1 state
		case 0x08: // I2C start condition has been generated on the bus -> transmit slave address and read/write bit -> next state: 0x18 or 0x40
			i2c1stat_prior_state = I2C1STAT;
			I2C1DAT
					= package_buffer1[i2c_package_buffer1_current_idx].slave_address
							| package_buffer1[i2c_package_buffer1_current_idx].direction;
			I2C1CONCLR = 1 << STAC; // do not restart (continue normal I2C operation)
			I2C1CONCLR = 1 << SIC; // clear I2C interrupt flag
			break;
		case 0x10: // "repeated start" condition has been generated on the bus -> transmit slave address and read/write bit -> next state: 0x18 or 0x40
			i2c1stat_prior_state = I2C1STAT;
			I2C1DAT
					= package_buffer1[i2c_package_buffer1_current_idx].slave_address
							| package_buffer1[i2c_package_buffer1_current_idx].direction;
			I2C1CONCLR = 1 << STAC;
			I2C1CONCLR = 1 << SIC;
			break;
		case 0x18: // slave address and write bit has been transmitted -> transmit first data byte -> next state: 0x28
			i2c1stat_prior_state = I2C1STAT;
			current_data_byte_i2c1 = 0;
			I2C1CONCLR = 1 << STAC;
			I2C1DAT
					= package_buffer1[i2c_package_buffer1_current_idx].data[current_data_byte_i2c1];
			current_data_byte_i2c1++;
			I2C1CONCLR = 1 << SIC;
			break;
		case 0x28: // data byte has been transmitted
			// if there's another byte to be transmitted, do it
			i2c1stat_prior_state = I2C1STAT;
			if (current_data_byte_i2c1
					< package_buffer1[i2c_package_buffer1_current_idx].length)
			{
				I2C1CONCLR = 1 << STAC;
				I2C1DAT
						= package_buffer1[i2c_package_buffer1_current_idx].data[current_data_byte_i2c1];
				current_data_byte_i2c1++;
				I2C1CONCLR = 1 << SIC;
			}
			// it the last byte has been transmitted, check if there are unhandled packages in the package buffer
			else
			{
				i2c_package_buffer1_current_idx
						= (i2c_package_buffer1_current_idx + 1)
								% I2C_PACKAGE_BUFFER_SIZE; // increment pointer to package in processing
				error_counter1 = 0;
				if (i2c_package_buffer1_current_idx
						== i2c_package_buffer1_insert_idx)
				{ // no unhandled packages
					I2C1CONCLR = 1 << STAC;
					I2C1CONSET = 1 << STO; // generate I2C stop condition on the bus without restart
					I2C1CONCLR = 1 << SIC; // clear I2C interrupt flag
					i2c1_busy = 0; // release I2C resource
				}
				else
				{ // unhandled packages in package buffer
					if (!(package_buffer1[i2c_package_buffer1_current_idx].write_read
							== 1))
					{
						I2C1CONSET = 1 << STO; // generate stop condition on the bus if no repeated start is necessary
					}
					I2C1CONSET = 1 << STA; // generate "repeated start" condition on the bus -> next state: 0x10
					I2C1CONCLR = 1 << SIC; // clear I2C interrupt flag
				}
			}
			break;
		case 0x20: // I2C error state detection
			i2c1stat_prior_state = I2C1STAT;
			I2C1CONSET = 1 << STO;
			I2C1CONSET = 1 << STA; // restart I2C state machine with current package
			error_counter1++;
			//debug_message_buffer("I2C error: slave address not acknowledged (write)\n");
			debug_message_buffer_sprintf(
					"I2C1 error: slave addr not ack (write). Dest: %i",
					package_buffer1[i2c_package_buffer1_current_idx].slave_address);

			//message_send_debug(COMM_1, buffer);
			I2C1CONCLR = 1 << SIC;
			break;
		case 0x30: // I2C error state detection
			i2c1stat_prior_state = I2C1STAT;
			I2C1CONSET = 1 << STO;
			I2C1CONSET = 1 << STA; // restart I2C state machine with current package
			error_counter1++;
			//		debug_message_buffer("I2C error: data not acknowledged\n");
			debug_message_buffer_sprintf(
					"I2C1 error: data not acknowledged. Dest: %i",
					package_buffer1[i2c_package_buffer1_current_idx].slave_address);

			//message_send_debug(COMM_1, buffer);
			I2C1CONCLR = 1 << SIC;
			break;
		case 0x38: // I2C error state detection
			i2c1stat_prior_state = I2C1STAT;
			I2C1CONSET = 1 << STA; // restart I2C state machine with current package
			error_counter1++;
			//		debug_message_buffer("I2C error: arbitration lost\n");
			debug_message_buffer_sprintf(
					"I2C1 error: arbitration lost. Dest: %i",
					package_buffer1[i2c_package_buffer1_current_idx].slave_address);

			//message_send_debug(COMM_1, buffer);
			I2C1CONCLR = 1 << SIC;
			break;
		case 0x40: // slave address and read bit has been transmitted -> clear interrupt and wait for first data byte -> next state: 0x50 or 0x58
			i2c1stat_prior_state = I2C1STAT;
			current_data_byte_i2c1 = 0;
			if (package_buffer1[i2c_package_buffer1_current_idx].length > 1)
				I2C1CONSET = 1 << AA; // if there's more than one byte to be received -> next state: 0x50
			else
				I2C1CONCLR = 1 << AAC; // if there's only one byte to be received -> next state: 0x58
			I2C1CONCLR = 1 << SIC; // clear I2C interrupt flag
			break;
		case 0x50: // data byte has been received
			i2c1stat_prior_state = I2C1STAT;
			package_buffer1[i2c_package_buffer1_current_idx].data[current_data_byte_i2c1]
					= I2C1DAT; // copy data byte to data array in I2C package
			current_data_byte_i2c1++; // increment data byte
			if ((current_data_byte_i2c1 + 1)
					< package_buffer1[i2c_package_buffer1_current_idx].length)
			{ // there's more than one byte left to be received
				I2C1CONSET = 1 << AA; // acknowledge next data byte -> next state: 0x50
			}
			else
			{ // there's only one byte left to be received
				I2C1CONCLR = 1 << AAC; // do not acknowledge next data byte -> next state: 0x58
			}
			I2C1CONCLR = 1 << SIC;
			break;
		case 0x58: // last data byte has been received
			i2c1stat_prior_state = I2C1STAT;
			package_buffer1[i2c_package_buffer1_current_idx].data[current_data_byte_i2c1]
					= I2C1DAT; // copy data byte to data array in I2C package
			I2C1CONSET = 1 << STO; // generate STOP condition on the I2C bus
			//uart0_transmit(package_buffer1[i2c_package_buffer1_current_idx].data[current_data_byte_i2c1]);
			if (package_buffer1[i2c_package_buffer1_current_idx].i2c_done_handler
					!= NULL)
			{ // check if there is a package handler registered
				package_buffer1[i2c_package_buffer1_current_idx].i2c_done_handler(
						&(package_buffer1[i2c_package_buffer1_current_idx])); // call package handler
			}
			i2c_package_buffer1_current_idx = (i2c_package_buffer1_current_idx
					+ 1) % I2C_PACKAGE_BUFFER_SIZE; // increment pointer to package in processing
			error_counter1 = 0;
			// check if there are unhandled packages in the package buffer
			if (i2c_package_buffer1_current_idx
					== i2c_package_buffer1_insert_idx)
			{ // no unhandled packages
				I2C1CONCLR = 1 << SIC; // clear I2C interrupt flag
				i2c1_busy = 0; // release I2C resource
			}
			else
			{ // unhandled packages in package buffer
				I2C1CONSET = 1 << STA; // generate "repeated start" condition on the bus -> next state: 0x10
				I2C1CONCLR = 1 << SIC; // clear I2C interrupt flag
			}
			break;
		case 0x48: // I2C error state detection
			i2c1stat_prior_state = I2C1STAT;
			I2C1CONSET = 1 << STO;
			I2C1CONSET = 1 << STA; // restart I2C state machine with current package
			error_counter1++;
			debug_message_buffer(
					"I2C1 error: slave address not acknowledged (read)\n");
			//message_send_debug(COMM_1, buffer);
			I2C1CONCLR = 1 << SIC;
			break;

		default: // I2C error state detection
			I2C1CONSET = 1 << STO;
			I2C1CONSET = 1 << STA; // restart I2C state machine with current package
			error_counter1++;
			debug_message_buffer_sprintf("I2C1 error: undefined I2C state: %i",I2C1STAT);
			debug_message_buffer_sprintf("I2C1 error: prior state: %X",i2c1stat_prior_state);
			//		message_send_debug(COMM_1, buffer);
			I2C1CONCLR = 1 << SIC;
		}
	}
// SEEMS NOT TO WORK
//	uint16_t timeout = 800;
//
//	while(!(I2C1CONSET & (1 << SI)))
//	{
//		timeout--;
//
//		if(!timeout) // timed out
//		{
//			debug_message_buffer("I2C wait while break\n");
//			break;
//		}
//	}

	VICVectAddr = 0x00000000; // clear this interrupt from the VIC

	// Sum up errors
	global_data.i2c1_err_count += error_counter1;

	ISR_EXIT();
	// exit ISR
}



/* copy & repalced 1 by 0 for I2C0 25.05.2010
 * updated 20100902 */
uint32_t i2c0_reject_count = 0;
/* I2C0 bus operation starter function */
void i2c0_op(i2c_package * package)
{
	//setup error codes
	//	package.i2c_error_code=I2C_CODE_NOT_KNOWN;
	//	package.i2c_error_counter=0;
	//TODO why doesn't this work?

	// If Buffer is full we can't send a package
	if (i2c_package_buffer0_current_idx - i2c_package_buffer0_insert_idx == 1
			|| (i2c_package_buffer0_current_idx == 0
					&& i2c_package_buffer0_insert_idx
							== (I2C_PACKAGE_BUFFER_SIZE - 1)))
	{
		//TODO Change function to return an error code and adapt all calls to check if they where successful. 20100428 Laurens

		// DIRTY HACK: a motorcontroller is pulling SDA down we will restart

				I2C0CONSET = 1 << STO; // generate I2C stop condition on the bus without restart
				I2C0CONSET = 1 << STA;
		//		I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag


		//		debug_message_buffer_sprintf(
		//			"i2c I2C0STAT is: %i.",
		//			I2C1STAT);

				debug_message_buffer_sprintf("*****Restarted I2C0*** I2C0STAT=%i",I2C0STAT);
		//END HACK

		if (i2c0_reject_count++ % 256 == 0)
		{
			debug_message_buffer_sprintf(
					"i2c0 buffer full. Rejected package. Total: %i",
					i2c0_reject_count);
		}
		return;
	}

	package_buffer0[i2c_package_buffer0_insert_idx] = *package; // add new i2c package to package buffer 0
	i2c_package_buffer0_insert_idx = (i2c_package_buffer0_insert_idx + 1)
			% I2C_PACKAGE_BUFFER_SIZE; // increment package insertion pointer

	if (!i2c0_busy)
	{ // check whether I2C0 is busy: if busy, data will I2C operation will be executed later, but program execution can continue
		i2c0_busy = 1; // process locks I2C resource
		I2C0CONSET = 1 << STA; // I2C start condition is generated on the bus -> interrupt is generated when ready; next state code: 0x08
		error_counter0 = 0;
	}

	return;
}

uint32_t i2c0_permanent_error_count = 0;
volatile int i2c0stat_prior_state = 0;
/* Interrupt handler for I2C0 interrupt */
static void I2C0_ISR(void)
{

	//sprintf((char*)buffer, "h ");
	//message_send_debug(COMM_1, buffer);
	//uint8_t buffer[200];	// string buffer for debug messages
	ISR_ENTRY();

	if (error_counter0 > I2C_PERMANENT_ERROR_LIMIT)
	{
		//uint8_t buffer[50];
		//sprintf(buffer, "i2c error limit %d \n", 1);
		//  sprintf((char*)buffer, package_buffer0[i2c_package_buffer0_current_idx].slave_address) ;
		//message_send_debug(COMM_1, buffer);
		//		debug_message_buffer("i2c error limit reached");
		debug_message_buffer_sprintf("i2c0 error limit reached. Dest: %i",
				package_buffer0[i2c_package_buffer0_current_idx].slave_address);

		if (i2c0_permanent_error_count++ % 256 == 0)
		{
			debug_message_buffer_sprintf(
					"i2c0 error limit reached. Total: %i errors.",
					i2c0_permanent_error_count);
		}

		package_buffer0[i2c_package_buffer0_current_idx].i2c_error_code
				= I2C_CODE_ERROR;//set error code to error
		//TODO set this to ok if everzthing was ok

		if (package_buffer0[i2c_package_buffer0_current_idx].i2c_done_handler
				!= NULL)
		{ // check if there is a package handler registered
			package_buffer0[i2c_package_buffer0_current_idx].i2c_done_handler(
					&(package_buffer0[i2c_package_buffer0_current_idx])); // call package handler
		}

		i2c_package_buffer0_insert_idx = (i2c_package_buffer0_insert_idx + 1)
				% I2C_PACKAGE_BUFFER_SIZE; // increment package insertion pointer
		error_counter0 = 0;
		if (i2c_package_buffer0_current_idx == i2c_package_buffer0_insert_idx)
		{ // no unhandled packages
			I2C0CONCLR = 1 << STAC;
			I2C0CONSET = 1 << STO; // generate I2C stop condition on the bus without restart
			I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
			i2c0_busy = 0; // release I2C resource
		}
		else
		{ // unhandled packages in package buffer
			if (!(package_buffer0[i2c_package_buffer0_current_idx].write_read
					== 1))
			{
				I2C0CONSET = 1 << STO; // generate stop condition on the bus if no repeated start is necessary
			}
			I2C0CONSET = 1 << STA; // generate "repeated start" condition on the bus -> next state: 0x10
			I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
		}
	}
	if (i2c0_busy == 1)
	{ //Return if Package error
		//message_debug_send(MAVLINK_COMM_1, 30, I2C0STAT);
		switch (I2C0STAT)
		{ // check current I2C0 state
		case 0x08: // I2C start condition has been generated on the bus -> transmit slave address and read/write bit -> next state: 0x18 or 0x40
			I2C0DAT
					= package_buffer0[i2c_package_buffer0_current_idx].slave_address
							| package_buffer0[i2c_package_buffer0_current_idx].direction;
			I2C0CONCLR = 1 << STAC; // do not restart (continue normal I2C operation)
			I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
			break;
		case 0x10: // "repeated start" condition has been generated on the bus -> transmit slave address and read/write bit -> next state: 0x18 or 0x40
			I2C0DAT
					= package_buffer0[i2c_package_buffer0_current_idx].slave_address
							| package_buffer0[i2c_package_buffer0_current_idx].direction;
			I2C0CONCLR = 1 << STAC;
			I2C0CONCLR = 1 << SIC;
			break;
		case 0x18: // slave address and write bit has been transmitted -> transmit first data byte -> next state: 0x28
			current_data_byte_i2c0 = 0;
			I2C0CONCLR = 1 << STAC;
			I2C0DAT
					= package_buffer0[i2c_package_buffer0_current_idx].data[current_data_byte_i2c0];
			current_data_byte_i2c0++;
			I2C0CONCLR = 1 << SIC;
			break;
		case 0x28: // data byte has been transmitted
			// if there's another byte to be transmitted, do it
			if (current_data_byte_i2c0
					< package_buffer0[i2c_package_buffer0_current_idx].length)
			{
				I2C0CONCLR = 1 << STAC;
				I2C0DAT
						= package_buffer0[i2c_package_buffer0_current_idx].data[current_data_byte_i2c0];
				current_data_byte_i2c0++;
				I2C0CONCLR = 1 << SIC;
			}
			// it the last byte has been transmitted, check if there are unhandled packages in the package buffer
			else
			{
				i2c_package_buffer0_current_idx
						= (i2c_package_buffer0_current_idx + 1)
								% I2C_PACKAGE_BUFFER_SIZE; // increment pointer to package in processing
				error_counter0 = 0;
				if (i2c_package_buffer0_current_idx
						== i2c_package_buffer0_insert_idx)
				{ // no unhandled packages
					I2C0CONCLR = 1 << STAC;
					I2C0CONSET = 1 << STO; // generate I2C stop condition on the bus without restart
					I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
					i2c0_busy = 0; // release I2C resource
				}
				else
				{ // unhandled packages in package buffer
					if (!(package_buffer0[i2c_package_buffer0_current_idx].write_read
							== 1))
					{
						I2C0CONSET = 1 << STO; // generate stop condition on the bus if no repeated start is necessary
					}
					I2C0CONSET = 1 << STA; // generate "repeated start" condition on the bus -> next state: 0x10
					I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
				}
			}
			break;
		case 0x20: // I2C error state detection
			I2C0CONSET = 1 << STO;
			I2C0CONSET = 1 << STA; // restart I2C state machine with current package
			error_counter0++;
			//debug_message_buffer("I2C error: slave address not acknowledged (write)\n");
			debug_message_buffer_sprintf(
					"I2C0 error: slave address not acknowledged (write). Dest: %i",
					package_buffer0[i2c_package_buffer0_current_idx].slave_address);

			//message_send_debug(COMM_1, buffer);
			I2C0CONCLR = 1 << SIC;
			break;
		case 0x30: // I2C error state detection
			I2C0CONSET = 1 << STO;
			I2C0CONSET = 1 << STA; // restart I2C state machine with current package
			error_counter0++;
			//		debug_message_buffer("I2C error: data not acknowledged\n");
			debug_message_buffer_sprintf(
					"I2C0 error: data not acknowledged. Dest: %i",
					package_buffer0[i2c_package_buffer0_current_idx].slave_address);

			//message_send_debug(COMM_1, buffer);
			I2C0CONCLR = 1 << SIC;
			break;
		case 0x38: // I2C error state detection
			I2C0CONSET = 1 << STA; // restart I2C state machine with current package
			error_counter0++;
			//		debug_message_buffer("I2C error: arbitration lost\n");
			debug_message_buffer_sprintf(
					"I2C0 error: arbitration lost. Dest: %i",
					package_buffer0[i2c_package_buffer0_current_idx].slave_address);

			//message_send_debug(COMM_1, buffer);
			I2C0CONCLR = 1 << SIC;
			break;
		case 0x40: // slave address and read bit has been transmitted -> clear interrupt and wait for first data byte -> next state: 0x50 or 0x58
			current_data_byte_i2c0 = 0;
			if (package_buffer0[i2c_package_buffer0_current_idx].length > 1)
				I2C0CONSET = 1 << AA; // if there's more than one byte to be received -> next state: 0x50
			else
				I2C0CONCLR = 1 << AAC; // if there's only one byte to be received -> next state: 0x58
			I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
			break;
		case 0x50: // data byte has been received
			package_buffer0[i2c_package_buffer0_current_idx].data[current_data_byte_i2c0]
					= I2C0DAT; // copy data byte to data array in I2C package
			current_data_byte_i2c0++; // increment data byte
			if ((current_data_byte_i2c0 + 1)
					< package_buffer0[i2c_package_buffer0_current_idx].length)
			{ // there's more than one byte left to be received
				I2C0CONSET = 1 << AA; // acknowledge next data byte -> next state: 0x50
			}
			else
			{ // there's only one byte left to be received
				I2C0CONCLR = 1 << AAC; // do not acknowledge next data byte -> next state: 0x58
			}
			I2C0CONCLR = 1 << SIC;
			break;
		case 0x58: // last data byte has been received
			package_buffer0[i2c_package_buffer0_current_idx].data[current_data_byte_i2c0]
					= I2C0DAT; // copy data byte to data array in I2C package
			I2C0CONSET = 1 << STO; // generate STOP condition on the I2C bus
			//uart0_transmit(package_buffer0[i2c_package_buffer0_current_idx].data[current_data_byte_i2c0]);
			if (package_buffer0[i2c_package_buffer0_current_idx].i2c_done_handler
					!= NULL)
			{ // check if there is a package handler registered
				package_buffer0[i2c_package_buffer0_current_idx].i2c_done_handler(
						&(package_buffer0[i2c_package_buffer0_current_idx])); // call package handler
			}
			i2c_package_buffer0_current_idx = (i2c_package_buffer0_current_idx
					+ 1) % I2C_PACKAGE_BUFFER_SIZE; // increment pointer to package in processing
			error_counter0 = 0;
			// check if there are unhandled packages in the package buffer
			if (i2c_package_buffer0_current_idx
					== i2c_package_buffer0_insert_idx)
			{ // no unhandled packages
				I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
				i2c0_busy = 0; // release I2C resource
			}
			else
			{ // unhandled packages in package buffer
				I2C0CONSET = 1 << STA; // generate "repeated start" condition on the bus -> next state: 0x10
				I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
			}
			break;
		case 0x48: // I2C error state detection
			I2C0CONSET = 1 << STO;
			I2C0CONSET = 1 << STA; // restart I2C state machine with current package
			error_counter0++;
			debug_message_buffer(
					"I2C0 error: slave address not acknowledged (read)\n");
			//message_send_debug(COMM_1, buffer);
			I2C0CONCLR = 1 << SIC;
			break;

		default: // I2C error state detection
			I2C0CONSET = 1 << STO;
			I2C0CONSET = 1 << STA; // restart I2C state machine with current package
			error_counter0++;
			debug_message_buffer_sprintf("I2C0 error: undefined I2C state: %i",I2C0STAT);
			debug_message_buffer_sprintf("I2C0 error: prior state: %X",i2c0stat_prior_state);
			//		message_send_debug(COMM_1, buffer);
			I2C0CONCLR = 1 << SIC;
		}
	}

	// Sum up errors
	global_data.i2c0_err_count += error_counter0;

	VICVectAddr = 0x00000000; // clear this interrupt from the VIC
	ISR_EXIT();
	// exit ISR
}

//DONT USE I2C0 !!!!!!!!!!!!!!!!!!!
//// has to be modified to look like i2c1!!!!!!!!!!!!!!!
//
///* I2C0 bus operation starter function */
//void i2c0_op(i2c_package * package)
//{
//	// wait for free space, if buffer is full
//	while (i2c_package_buffer0_current_idx - i2c_package_buffer0_insert_idx
//			== 1 || (i2c_package_buffer0_current_idx == 0
//			&& i2c_package_buffer0_insert_idx == (I2C_PACKAGE_BUFFER_SIZE - 1)))
//		;
//
//	package_buffer0[i2c_package_buffer0_insert_idx] = *package; // add new i2c package to package buffer 0
//	i2c_package_buffer0_insert_idx = (i2c_package_buffer0_insert_idx + 1)
//			% I2C_PACKAGE_BUFFER_SIZE; // increment package insertion pointer
//
//	if (!i2c0_busy)
//	{ // check whether I2C0 is busy: if busy, data will I2C operation will be executed later, but program execution can continue
//		i2c0_busy = 1; // process locks I2C resource
//		I2C0CONSET = 1 << STA; // I2C start condition is generated on the bus -> interrupt is generated when ready; next state code: 0x08
//		error_counter0 = 0;
//	}
//
//	return;
//}
//
///* Interrupt handler for I2C0 interrupt */
//static void I2C0_ISR(void)
//{
//	ISR_ENTRY();
//	// enter ISR
//
//	//char buffer[50];	// string buffer for debug messages
//	if (error_counter0 > I2C_PERMANENT_ERROR_LIMIT)
//	{
//		debug_message_buffer("Permanent I2C error\n");
//	}
//
//	switch (I2C0STAT)
//	{ // check current I2C0 state
//	case 0x08: // I2C start condition has been generated on the bus -> transmit slave address and read/write bit -> next state: 0x18 or 0x40
//		I2C0DAT
//				= package_buffer0[i2c_package_buffer0_current_idx].slave_address
//						| package_buffer0[i2c_package_buffer0_current_idx].direction;
//		I2C0CONCLR = 1 << STAC;
//		I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
//		break;
//	case 0x10: // "repeated start" condition has been generated on the bus -> transmit slave address and read/write bit -> next state: 0x18 or 0x40
//		I2C0DAT
//				= package_buffer0[i2c_package_buffer0_current_idx].slave_address
//						| package_buffer0[i2c_package_buffer0_current_idx].direction;
//		I2C0CONCLR = 1 << STAC;
//		I2C0CONCLR = 1 << SIC;
//		break;
//	case 0x18: // slave address and write bit has been transmitted -> transmit first data byte -> next state: 0x28
//		current_data_byte_i2c0 = 0;
//		I2C0CONCLR = 1 << STAC;
//		I2C0DAT
//				= package_buffer0[i2c_package_buffer0_current_idx].data[current_data_byte_i2c0];
//		current_data_byte_i2c0++;
//		I2C0CONCLR = 1 << SIC;
//		break;
//	case 0x28: // data byte has been transmitted
//		// if there's another byte to be transmitted, do it
//		if (current_data_byte_i2c0
//				< package_buffer0[i2c_package_buffer0_current_idx].length)
//		{
//			I2C0CONCLR = 1 << STAC;
//			I2C0DAT
//					= package_buffer0[i2c_package_buffer0_current_idx].data[current_data_byte_i2c0];
//			current_data_byte_i2c0++;
//			I2C0CONCLR = 1 << SIC;
//		}
//		// it the last byte has been transmitted, check if there are unhandled packages in the package buffer
//		else
//		{
//			i2c_package_buffer0_current_idx = (i2c_package_buffer0_current_idx
//					+ 1) % I2C_PACKAGE_BUFFER_SIZE; // increment pointer to package in processing
//			error_counter0 = 0;
//			if (i2c_package_buffer0_current_idx
//					== i2c_package_buffer0_insert_idx)
//			{ // no unhandled packages
//				I2C0CONCLR = 1 << STAC;
//				I2C0CONSET = 1 << STO; // generate I2C stop condition on the bus without restart
//				I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
//				i2c0_busy = 0; // release I2C resource
//			}
//			else
//			{ // unhandled packages in package buffer
//				if (!(package_buffer0[i2c_package_buffer1_current_idx].write_read
//						== 1))
//				{
//					I2C0CONSET = 1 << STO; // generate stop condition on the bus if no repeated start is necessary
//				}
//				I2C0CONSET = 1 << STA; // generate "repeated start" condition on the bus -> next state: 0x10
//				I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
//
//			}
//		}
//		break;
//	case 0x20: // I2C error state detection
//		I2C0CONSET = 1 << STO;
//		I2C0CONSET = 1 << STA; // restart I2C state machine with current package
//		error_counter0++;
//		debug_message_buffer(
//				"I2C error: slave address not acknowledged (write)\n");
//		I2C0CONCLR = 1 << SIC;
//		break;
//	case 0x30: // I2C error state detection
//		I2C0CONSET = 1 << STO;
//		I2C0CONSET = 1 << STA; // restart I2C state machine with current package
//		error_counter0++;
//		debug_message_buffer("I2C error: data not acknowledged\n");
//		I2C0CONCLR = 1 << SIC;
//		break;
//	case 0x38: // I2C error state detection
//		I2C0CONSET = 1 << STA; // restart I2C state machine with current package
//		error_counter0++;
//		debug_message_buffer("I2C error: arbitration lost\n");
//		I2C0CONCLR = 1 << SIC;
//		break;
//	case 0x40: // slave address and read bit has been transmitted -> clear interrupt and wait for first data byte -> next state: 0x50 or 0x58
//		current_data_byte_i2c0 = 0;
//		if (package_buffer0[i2c_package_buffer0_current_idx].length > 1)
//			I2C0CONSET = 1 << AA; // if there's more than one byte to be received -> next state: 0x50
//		else
//			I2C0CONCLR = 1 << AAC; // if there's only one byte to be received -> next state: 0x58
//		I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
//		break;
//	case 0x50: // data byte has been received
//		package_buffer0[i2c_package_buffer0_current_idx].data[current_data_byte_i2c0]
//				= I2C0DAT; // copy data byte to data array in I2C package
//		current_data_byte_i2c0++; // increment data byte
//		if ((current_data_byte_i2c0 + 1)
//				< package_buffer0[i2c_package_buffer0_current_idx].length)
//		{ // there's more than one byte left to be received
//			I2C0CONSET = 1 << AA; // acknowledge next data byte -> next state: 0x50
//		}
//		else
//		{ // there's only one byte left to be received
//			I2C0CONCLR = 1 << AAC; // do not acknowledge next data byte -> next state: 0x58
//		}
//		I2C0CONCLR = 1 << SIC;
//		break;
//	case 0x58: // last data byte has been received
//
//		package_buffer0[i2c_package_buffer0_current_idx].data[current_data_byte_i2c0]
//				= I2C0DAT; // copy data byte to data array in I2C package
//		I2C0CONSET = 1 << STO; // generate STOP condition on the I2C bus
//		// execute I2C package processing function defined by driver for specific I2C devices
//		if (package_buffer0[i2c_package_buffer0_current_idx].i2c_done_handler
//				!= NULL)
//		{ // check if there is a package handler registered
//			package_buffer0[i2c_package_buffer0_current_idx].i2c_done_handler(
//					&(package_buffer0[i2c_package_buffer0_current_idx])); // call package handler
//		}
//		i2c_package_buffer0_current_idx = (i2c_package_buffer0_current_idx + 1)
//				% I2C_PACKAGE_BUFFER_SIZE; // increment pointer to package in processing
//		error_counter0 = 0;
//		// check if there are unhandled packages in the package buffer
//		if (i2c_package_buffer0_current_idx == i2c_package_buffer0_insert_idx)
//		{ // no unhandled packages
//			I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
//			i2c0_busy = 0; // release I2C resource
//		}
//		else
//		{ // unhandled packages in package buffer
//			I2C0CONSET = 1 << STA; // generate "repeated start" condition on the bus -> next state: 0x10
//			I2C0CONCLR = 1 << SIC; // clear I2C interrupt flag
//		}
//		break;
//	case 0x48: // I2C error state detection
//		I2C0CONSET = 1 << STO;
//		I2C0CONSET = 1 << STA; // restart I2C state machine with current package
//		error_counter0++;
//		//sprintf((char *)buffer, "I2C error: slave address not acknowledged (read)\n");
//		debug_message_buffer(
//				"I2C error: slave address not acknowledged (read)\n");
//		//message_send_debug(COMM_1, buffer);
//		I2C0CONCLR = 1 << SIC;
//		break;
//
//	default: // I2C error state detection
//		I2C0CONSET = 1 << STO;
//		I2C0CONSET = 1 << STA; // restart I2C state machine with current package
//		error_counter0++;
//		debug_message_buffer("I2C error: undefined I2C state\n");
//		//message_send_debug(COMM_1, buffer);
//		I2C0CONCLR = 1 << SIC;
//	}
//
//	VICVectAddr = 0x00000000; // clear this interrupt from the VIC
//	ISR_EXIT();
//	// exit ISR
//}

#endif
