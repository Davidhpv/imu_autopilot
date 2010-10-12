/*
 * scp1000.c
 *
 *  Created on: 11.05.2009
 *      Author: Martin Rutschmann
 */

#include "ads8341.h"
#include "armVIC.h"
#include "LPC21xx.h"
#include "conf.h"
#include "spi.h"

unsigned int ads8341_value;

static void SSP_ISR(void) __attribute__((naked));
void ads8341_spi_init(void);
void ads8341_unselect(void);
void ads8341_select(void);
void ads8341_on_spi_int(void);


void ads8341_spi_init(void){

	/* setup pins for SSP (SCK, MISO, MOSI) */
	PINSEL1 |= SSP_PINSEL1_SCK  | SSP_PINSEL1_MISO | SSP_PINSEL1_MOSI;

	/* setup SSP */
	SSPCR0 = SSPCR0_VAL;
	SSPCR1 = SSPCR1_VAL;
	SSPCPSR = 0x04;

	/* initialize interrupt vector */
	VICIntSelect &= ~VIC_BIT( VIC_SPI1 );  /* SPI1 selected as IRQ */
	VICIntEnable = VIC_BIT( VIC_SPI1 );    /* enable it            */
	_VIC_CNTL(SSP_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
	_VIC_ADDR(SSP_VIC_SLOT) = (unsigned int)SSP_ISR;      /* address of the ISR   */

}

void ads8341_init(void) {

	/* configure SS pin */
	ADS8341_SS_IODIR |= 1 << ADS8341_SS_PIN; /* pin is output  */
	ads8341_unselect(); /* pin idles high */

	ads8341_spi_init();

}

void ads8341_unselect(void) {
	ADS8341_SS_IOSET |= 1 << ADS8341_SS_PIN;
}
void ads8341_select(void) {
	ADS8341_SS_IOCLR |= 1 << ADS8341_SS_PIN;
}

void ads8341_read(int channel) {
	unsigned char cmd1 = (1<<4);
	switch (channel){
	case 0:
		cmd1 |= (1<<1);
		break;
	case 1:
		cmd1 |= (1<<3 | 1<<1);
		break;
	case 2:
		cmd1 |= (1<<2);
		break;
	case 3:
		cmd1 |= (1<<3 | 1<<2);
		break;
	default:
		return;
	}
	unsigned char cmd2 = (1<<4);
	ads8341_select();
	SpiClearRti();
	SpiEnableRti();
	SSPDR = cmd1;
	SSPDR = cmd2;
	SSPDR = 0;
	SSPDR = 0;
	SSPDR = 0;
	SpiEnable();
}

void ads8341_on_spi_int(void) {
	unsigned char foo __attribute__ ((unused)) = SSPDR;
	unsigned int data = (SSPDR&1)<<15;
	data += SSPDR<<10;
	data += SSPDR<<5;
	data += SSPDR<<0;
	ads8341_value = data;
}

static void SSP_ISR(void) {
	ISR_ENTRY();
	ads8341_on_spi_int();
	ads8341_unselect();
	VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
	ISR_EXIT();
}

unsigned int ads8341_get_value(int channel){
	return ads8341_value;
}

