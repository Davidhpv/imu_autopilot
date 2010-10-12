/*
 * scp1000.c
 *
 *  Created on: 11.05.2009
 *      Author: Martin Rutschmann
 */

#include "scp1000.h"
#include "armVIC.h"
#include "LPC21xx.h"
#include "conf.h"
#include "spi.h"

unsigned int scp1000_pressure;
int scp1000_status;
int scp1000_statusreg;

static void EXTINT_ISR(void) __attribute__((naked));
static void SSP_ISR(void) __attribute__((naked));
void scp1000_spi_init(void);
void scp1000_unselect(void);
void scp1000_select(void);
void scp1000_on_spi_int(void);


void scp1000_spi_init(void){

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

void scp1000_init(void) {

	scp1000_status = SCP1000_STA_STOPPED;

	/* configure SS pin */
	SCP_SS_IODIR |= 1 << SCP_SS_PIN; /* pin is output  */
	scp1000_unselect(); /* pin idles high */

	scp1000_spi_init();

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

void scp1000_unselect(void) {
	SCP_SS_IOSET |= 1 << SCP_SS_PIN;
}
void scp1000_select(void) {
	SCP_SS_IOCLR |= 1 << SCP_SS_PIN;
}

void scp1000_send_config(unsigned char data) {
	scp1000_status = SCP1000_STA_STOPPED;
	const unsigned char cmd = 0x03 << 2 | 0x02;
	scp1000_select();
	SpiClearRti();
	SpiEnableRti();
	SSPDR = cmd;
	SSPDR = data;
	SpiEnable();
}

void scp1000_read(void) {
	const unsigned char cmd1 = 0x1F << 2;
	const unsigned char cmd2 = 0x20 << 2;
	scp1000_select();
	scp1000_status = SCP1000_STA_SENDING_REQUEST;
	SpiClearRti();
	SpiEnableRti();
	SSPDR = cmd1;
	SSPDR = 0;
	SSPDR = cmd2;
	SSPDR = 0;
	SSPDR = 0;
	SpiEnable();
}

void scp1000_read_statusreg(void){
	scp1000_status = SCP1000_STA_READ_STATUSREG;
	const unsigned char cmd = 0x07 << 2;
	scp1000_select();
	SpiClearRti();
	SpiEnableRti();
	SSPDR = cmd;
	SSPDR = 0;
	SpiEnable();
}

void scp1000_send_software_reset(void){
	scp1000_status = SCP1000_STA_STOPPED;
	const unsigned char cmd = 0x06 << 2 | 2;
	const unsigned char data = 1;
	scp1000_select();
	SpiClearRti();
	SpiEnableRti();
	SSPDR = cmd;
	SSPDR = data;
	SpiEnable();
}

int scp1000_get_statusreg(void){
	return scp1000_statusreg;
}


/* FIXME READ high bit last */

void scp1000_on_spi_int(void) {
	switch (scp1000_status) {
	case SCP1000_STA_STOPPED: {
		unsigned char foo1 __attribute__ ((unused)) = SSPDR;
		unsigned char foo2 __attribute__ ((unused)) = SSPDR;
		scp1000_unselect();
		SpiClearRti();
		SpiDisableRti();
		SpiDisable();
		scp1000_status = SCP1000_STA_WAIT_EOC;
	}
		break;
	case SCP1000_STA_SENDING_REQUEST: {
		unsigned char foo1 __attribute__ ((unused)) = SSPDR;
		unsigned int high_bit = ((SSPDR) & 0x3) << 16;
		unsigned char foo2 __attribute__ ((unused)) = SSPDR;
		scp1000_pressure = SSPDR << 8;
		scp1000_pressure += SSPDR;
		scp1000_pressure += high_bit;
		scp1000_unselect();
		SpiClearRti();
		SpiDisableRti();
		SpiDisable();
		scp1000_status = SCP1000_STA_DATA_AVAILABLE;
	}
		break;
	case SCP1000_STA_READ_STATUSREG: {
		unsigned char foo1 __attribute__ ((unused)) = SSPDR;
		scp1000_statusreg = SSPDR;
		scp1000_unselect();
		SpiClearRti();
		SpiDisableRti();
		SpiDisable();
	}
		break;
	}
}

static void SSP_ISR(void) {
	ISR_ENTRY();
	scp1000_on_spi_int();
	VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
	ISR_EXIT();
}

void EXTINT_ISR(void) {
	ISR_ENTRY();

	scp1000_status = SCP1000_STA_GOT_EOC;
	//scp1000_read();

	EXTINT |= 1 << SCP_DRDY_EINT; /* clear EINT */
	VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
	ISR_EXIT();
}

unsigned int scp1000_get_value(void){
	return scp1000_pressure;
}

