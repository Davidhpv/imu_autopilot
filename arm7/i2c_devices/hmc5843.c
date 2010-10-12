/*
 * hmc5843.c
 *
 *  Created on: 13.07.2010
 *      Author: Laurens Mackay
 */

#include "conf.h"
#include "hmc5843.h"
#include "armVIC.h"
#include "inttypes.h"
#include "global_data.h"
#include "string.h"
#include "i2c.h"
#include "debug.h"

uint8_t hmc5843_address = 0x3c;
uint8_t hmc5843_bus = 1;
int16_vect3 hmc5843_result;
int8_t hmc5843_data_valid = 0;

static void EXTINT_ISR(void) __attribute__((naked));

void hmc5843_init()
{
	/* configure DRDY pin */
	/* connected pin to EXINT */
	HMC5843_DRDY_PINSEL |= HMC5843_DRDY_PINSEL_VAL << HMC5843_DRDY_PINSEL_BIT;
	EXTMODE|= 1 << HMC5843_DRDY_EINT; /* EINT is edge trigered */
	EXTPOLAR|= 1 << HMC5843_DRDY_EINT; /* EINT is trigered on rising edge */
	EXTINT|=  1 << HMC5843_DRDY_EINT; /* clear pending EINT */

	/* initialize interrupt vector */
	VICIntSelect &= ~VIC_BIT(HMC5843_DRDY_VIC_IT); /* select EINT as IRQ source */
	VICIntEnable = VIC_BIT(HMC5843_DRDY_VIC_IT); /* enable it                 */
	_VIC_CNTL( MAG_DRDY_VIC_SLOT) = VIC_ENABLE | HMC5843_DRDY_VIC_IT;
	_VIC_ADDR( MAG_DRDY_VIC_SLOT) = (unsigned int) EXTINT_ISR; // address of the ISR


	// The HMC5843 has 13 registers for configuration and
	// data readout, indexed from 00 to 12
	//
	// 00		Configuration Register A		R/W
	// 01		Configuration Register B		R/W
	// 02		Mode Register					R/W
	// 03-08	Data Output Register			R
	// 09		Status Register					R
	// 10		Identification Register A		R
	// 11		Identification Register B		R
	// 12		Identification Register C		R

	// The configuration for register A is (default mode from datasheet)
	//
	// Normal measurement mode:					MS1 = 0, MS0 = 0
	// Minimum data output rate at 50 Hz:		DO2 = 1, DO1 = 1, DO0 = 0
	//											CRA7 = 0, CRA6 = 0, CRA5 = 0
	//
	// Resulting bitfield: 0b00011000 = 0x18

	// The configuration for register B is:
	//
	// Sensor input range: +- 1.0Ga (default):  GN2 = 0, GN1 = 0, GN0 = 1

	// Enable continous measurement mode
	i2c_package package_write;
	package_write.data[0] = 0x02;			// Select mode register (index 2, see table above)
	package_write.data[1] = 0x00;			// Write all bits zero to enable continuous measurement mode
	package_write.length = 2;
	package_write.direction = I2C_WRITE;
	package_write.slave_address = hmc5843_address;
	package_write.bus_number = hmc5843_bus;
	package_write.write_read = 0;
	package_write.i2c_done_handler = NULL;
	i2c_op(&package_write);

	// Set output rate to 50 Hz
	package_write.data[0] = 0x00;			// Select configuration register A (index 0, see table above)
	package_write.data[1] = 0x18;			// Resulting bitfield for 50Hz and normal modes: 0b00011000 = 0x18
	package_write.length = 2;
	package_write.direction = I2C_WRITE;
	package_write.slave_address = hmc5843_address;
	package_write.bus_number = hmc5843_bus;
	package_write.write_read = 0;
	package_write.i2c_done_handler = NULL;
	i2c_op(&package_write);
}
void hmc5843_start_read()
{
	i2c_package package_read, package_write;
	package_write.data[0] = 0x03;
	package_write.length = 1;
	package_write.direction = I2C_WRITE;
	package_write.slave_address = hmc5843_address;
	package_write.bus_number = hmc5843_bus;
	package_write.write_read = 1;
	package_write.i2c_done_handler = NULL;

	package_read.length = 6;
	package_read.direction = I2C_READ;
	package_read.slave_address = hmc5843_address + 1;
	package_read.bus_number = hmc5843_bus;
	package_read.write_read = 1; // make repeated start after this package to receive data
	package_read.i2c_done_handler = (void*) &hmc5843_read_handler;

	i2c_write_read(&package_write, &package_read);
	//i2c_op(&package_read);
}

void hmc5843_read_handler(i2c_package *package)
{
	if (package->i2c_error_code != I2C_CODE_ERROR)
	{
		hmc5843_result.x = twos_complement_decode16((package->data[0] << 8)
				| package->data[1]);
		hmc5843_result.y = twos_complement_decode16((package->data[2] << 8)
				| package->data[3]);
		hmc5843_result.z = twos_complement_decode16((package->data[4] << 8)
				| package->data[5]);

		//Check if the sensors are saturated
		if (hmc5843_result.x == -4096 || hmc5843_result.y == -4096
				|| hmc5843_result.z == -4096)
		{
			hmc5843_data_valid = 0;
		}
		else
		{
			hmc5843_data_valid = 1;
		}
	}
	else
	{
		hmc5843_data_valid = 0;
	}
}

void hmc5843_get_data(int16_vect3 *value)
{
	value->x = hmc5843_result.x;
	value->y = hmc5843_result.y;
	value->z = hmc5843_result.z;
}

int8_t hmc5843_data_ok()
{
	return hmc5843_data_valid;
}

int16_t twos_complement_decode16(int16_t twos_complement)
{
	int16_t value;
	if (twos_complement & (1 << 15))
	{
		value = ((int16_t) (twos_complement & 0x7FFF) - 32768);
	}
	else
	{
		value = (int16_t) twos_complement;
	}
	return value;
}

static void EXTINT_ISR(void)
{
	ISR_ENTRY();

	hmc5843_start_read();

	/* clear EINT */
	EXTINT|=  1 << HMC5843_DRDY_EINT;

	VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
	ISR_EXIT();
}
