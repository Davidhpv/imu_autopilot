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
*
*   @brief Driver for SCP1000 digital pressure sensor
*   @author Martin Rutschmann <pixhawk@student.ethz.ch>
*
*/

#include "scp1000.h"
#include "armVIC.h"
#include "LPC21xx.h"
#include "conf.h"
#include "spi.h"
#include "led.h"

unsigned int scp1000_pressure;
static spi_package scp1000_conf_package;
static spi_package scp1000_read_package;

/* Interrupt routines */
static void EXTINT_ISR(void) __attribute__((naked));

static void scp1000_unselect(void);
static void scp1000_select(void);
static void scp1000_on_spi_int_conf(void);
static void scp1000_on_spi_int_read(void);
static inline void scp1000_read(void);


void scp1000_init(void) {

	scp1000_conf_package.bit_mode = SPI_8_BIT_MODE;
	scp1000_conf_package.data[0] =  0x03 << 2 | 0x02;
	scp1000_conf_package.data[1] = SCP_MEAS_MODE;
	scp1000_conf_package.length = 2;
	scp1000_conf_package.slave_select = &scp1000_select;
	scp1000_conf_package.slave_unselect = &scp1000_unselect;
	scp1000_conf_package.spi_interrupt_handler = &scp1000_on_spi_int_conf;

	scp1000_read_package.bit_mode = SPI_8_BIT_MODE;
	scp1000_read_package.data[0] = 0x1F << 2;
	scp1000_read_package.data[1] = 0;
	scp1000_read_package.data[2] = 0x20 << 2;
	scp1000_read_package.data[3] = 0;
	scp1000_read_package.data[4] = 0;
	scp1000_read_package.length = 5;
	scp1000_read_package.slave_select = &scp1000_select;
	scp1000_read_package.slave_unselect = &scp1000_unselect;
	scp1000_read_package.spi_interrupt_handler = &scp1000_on_spi_int_read;

	/* configure SS pin */
	SCP_SS_IODIR |= 1 << SCP_SS_PIN; /* pin is output  */
	scp1000_unselect(); /* pin idles high */

	/* configure DRDY pin */
	/* connected pin to EXINT */
	SCP_DRDY_PINSEL |= SCP_DRDY_PINSEL_VAL << SCP_DRDY_PINSEL_BIT;
	EXTMODE |= 1 << SCP_DRDY_EINT; /* EINT is edge trigered */
	EXTPOLAR |= 1 << SCP_DRDY_EINT; /* EINT is trigered on rising edge */
	EXTINT |= 1 << SCP_DRDY_EINT; /* clear pending EINT */

	/* initialize interrupt vector */
	VICIntSelect &= ~VIC_BIT(SCP_DRDY_VIC_IT); /* select EINT as IRQ source */
	VICIntEnable = VIC_BIT(SCP_DRDY_VIC_IT); /* enable it */
	_VIC_CNTL( SCP1000_EOC_VIC_SLOT) = VIC_ENABLE | SCP_DRDY_VIC_IT;
	_VIC_ADDR( SCP1000_EOC_VIC_SLOT) = (unsigned int) EXTINT_ISR; // address of the ISR

}

void scp1000_send_config(void) {
	/* send the configuration data to the scp1000 */
	spi_transmit(&scp1000_conf_package);
}

static inline void scp1000_read(void) {
	/* read out data from the scp1000 */
	spi_transmit(&scp1000_read_package);
}

static void scp1000_unselect(void) {
	/* unselect the scp1000 by putting slaveselect high */
	SCP_SS_IOSET |= 1 << SCP_SS_PIN;
}
static void scp1000_select(void) {
	/* select the scp1000 by putting slaveselect low */
	SCP_SS_IOCLR |= 1 << SCP_SS_PIN;
}

static void scp1000_on_spi_int_conf(void) {
	/* read out dummy frames from send */
	unsigned char foo1 __attribute__ ((unused)) = SSPDR;
	unsigned char foo2 __attribute__ ((unused)) = SSPDR;
}

static void scp1000_on_spi_int_read(void) {
	/* read out dummy frame from first send byte */
	unsigned char foo1 __attribute__ ((unused)) = SSPDR;
	/* read out most significant byte */
	scp1000_pressure = SSPDR << 16;
	/* read out dummy frame from second send byte */
	unsigned char foo2 __attribute__ ((unused)) = SSPDR;
	/* read out middle byte */
	scp1000_pressure += SSPDR << 8;
	/* read out less significant byte */
	scp1000_pressure += SSPDR;
}

void EXTINT_ISR(void) {
	ISR_ENTRY();

	/* scp1000 has measured new data, now it's time to read it from spi */
	scp1000_read();

	EXTINT |= 1 << SCP_DRDY_EINT; /* clear EINT */
	VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
	ISR_EXIT();
}

unsigned int scp1000_get_value(void){
	return scp1000_pressure;
}

