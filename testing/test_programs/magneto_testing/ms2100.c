/*
 * micromag_testing.c
 *
 *  Created on: May 14, 2009
 *      Author: mavteam
 */


#include "comm_temp.h"
#include "spi.h"
#include "ms2100.h"
#include "armVIC.h"
#include "conf.h"
#include <math.h>

static void EXTINT_ISR(void) __attribute__((naked));
static void SSP_ISR(void) __attribute__((naked));
void micromag_spi_init(void);
void MmOnSpiIt(void);

int micromag_status;


void micromag_spi_init(void){

	/* setup pins for SSP (SCK, MISO, MOSI) */
	PINSEL1 |= SSP_PINSEL1_SCK  | SSP_PINSEL1_MISO | SSP_PINSEL1_MOSI;

	/* setup SSP */
	SSPCR0 = SSPCR0_VAL;
	SSPCR1 = SSPCR1_VAL;
	SSPCPSR = 0x02;

	/* initialize interrupt vector */
	VICIntSelect &= ~VIC_BIT( VIC_SPI1 );  /* SPI1 selected as IRQ */
	VICIntEnable = VIC_BIT( VIC_SPI1 );    /* enable it            */
	_VIC_CNTL(SSP_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
	_VIC_ADDR(SSP_VIC_SLOT) = (unsigned int)SSP_ISR;      /* address of the ISR   */

}

void micromag_init(void) {
	micromag_spi_init();

	//HACK set SS of max1168 to high
	IO0DIR|=1<<20;//set as output
	IO0SET|=1<<20;//set to high

	/* configure SS pin */
	MM_SS_IODIR|=1<<MM_SS_PIN; /* pin is output  */
	MmUnselect(); /* pin idles high */

	/* configure RESET pin */
	MM_RESET_IODIR|=1<<MM_RESET_PIN; /* pin is output  */
	MmReset(); /* pin idles low  */

	/* configure DRDY pin */
	/* connected pin to EXINT */
	MM_DRDY_PINSEL |= MM_DRDY_PINSEL_VAL << MM_DRDY_PINSEL_BIT;
	EXTMODE|= MM_DRDY_EINT; /* EINT is edge trigered */
	EXTPOLAR|= MM_DRDY_EINT; /* EINT is trigered on rising edge */
	EXTINT|=  MM_DRDY_EINT; /* clear pending EINT */

	/* initialize interrupt vector */
	VICIntSelect &= ~VIC_BIT(MM_DRDY_VIC_IT); /* select EINT as IRQ source */
	VICIntEnable = VIC_BIT(MM_DRDY_VIC_IT); /* enable it                 */
	_VIC_CNTL( MICROMAG_DRDY_VIC_SLOT) = VIC_ENABLE | MM_DRDY_VIC_IT;
	_VIC_ADDR( MICROMAG_DRDY_VIC_SLOT) = (unsigned int) EXTINT_ISR; // address of the ISR
}



void MmReadRes(void) {
	//comm_send_string("MmReadRes\r\n");
    micromag_status = MM_READING_RES; //set status
    MmSelect(); //set slave select pin to ground
    SpiClearRti(); //clear spi receive timeout interrupt
    SpiEnableRti(); //enable spi receive timeout interrupt
    /* trigger 2 bytes read */
    SSPDR = 0;
    SSPDR = 0;
    SpiEnable(); //enable spi
}

void MmSendReq(int axis) {
	MmSelect(); //set slave select pin to ground
	MmSet();
	micromag_status = MM_SENDING_REQ; //set status
	MmReset();
	comm_send_string("MmSendReq for axis: ");
	comm_send_int(axis);
	comm_send_string("\r\n");
	unsigned char control_byte = (axis) << 0 | 3 << 4; //make control byte to read current axis (ratio=/256)
	SSPDR = control_byte; //write control byte into send fifo
	SpiClearRti(); //clear the receive timeout interrupt
	SpiEnableRti(); //enable the receive timeout interrupt
	SpiEnable(); //enable spi and send control byte
}

void MmSendMOT(void) {
//	comm_send_string("MmSendMOT: ");
//	comm_send_string("\r\n");
	micromag_status = MM_SENDING_REQ; //set status
	MmSelect(); //set slave select pin to ground
	SpiClearRti(); //clear the receive timeout interrupt
	SpiEnableRti(); //enable the receive timeout interrupt
	unsigned char control_byte = 1 << 0 | 4 << 4 | 1 << 2; //make control byte to read current axis (ratio=/256)
	SSPDR = control_byte; //write control byte into send fifo
	SpiEnable(); //enable spi and send control byte
}

void MmOnSpiIt(void) {
	comm_send_string("mmOnSpiIt\r\n");
	if (micromag_status == MM_SENDING_REQ) {
			comm_send_string("1case MM_SENDING_REQ\r\n");
			/* read dummy control byte reply */
			unsigned char foo __attribute__ ((unused)) = SSPDR;
			micromag_status = MM_WAITING_EOC;
			SpiClearRti();
			SpiDisableRti();
			SpiDisable();
			MmUnselect();
			return;
		}
	if (micromag_status == MM_READING_RES){
			comm_send_string("2case MM_READING_RES:\r\n");

			new_val = SSPDR << 8;
			new_val += SSPDR;

//			comm_send_string(": ");
//			comm_send_int(new_val);
//			comm_send_string("\r\n");

			SpiClearRti();
			SpiDisableRti();
			SpiDisable();
			MmUnselect();

			return;
	}
}

static void SSP_ISR(void) {
	ISR_ENTRY();
	MmOnSpiIt();
	VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
	ISR_EXIT();
}

static void EXTINT_ISR(void) {
	ISR_ENTRY();
	comm_send_string("EXTINT_ISR\r\n");

    micromag_status = MM_GOT_EOC;
	MmReadRes();

	/* clear EINT */
	EXTINT|=  MM_DRDY_EINT;

	VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
	ISR_EXIT();
}
