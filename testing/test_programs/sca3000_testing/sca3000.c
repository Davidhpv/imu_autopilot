/*
 * micromag_testing.c
 *
 *  Created on: May 14, 2009
 *      Author: mavteam
 */


#include "comm_temp.h"
#include "spi.h"
#include "sca3000.h"
#include "armVIC.h"
#include "conf.h"

//static void EXTINT_ISR(void) __attribute__((naked));
static void SSP_ISR(void) __attribute__((naked));
void sca3000_spi_init(void);
void sca3000_on_spi_int(void);



int sca3000_status;

void sca3000_spi_init(void){

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

void sca3000_init(void) {
	sca3000_spi_init();

	/* configure SS pin */
	SCA3000_SS_IODIR|=1<<SCA3000_SS_PIN; /* pin is output  */
	sca3000_unselect(); /* pin idles high */


//	/* configure DRDY pin */
//	/* connected pin to EXINT */
//	SCA3000_DRDY_PINSEL |= SCA3000_DRDY_PINSEL_VAL << SCA3000_DRDY_PINSEL_BIT;
//	EXTMODE|= 1<<SCA3000_DRDY_EINT; /* EINT is edge trigered */
//	EXTPOLAR|= 1<<SCA3000_DRDY_EINT; /* EINT is trigered on rising edge */
//	EXTINT|=  1<<SCA3000_DRDY_EINT; /* clear pending EINT */
//
//	/* initialize interrupt vector */
//	VICIntSelect &= ~VIC_BIT(SCA3000_DRDY_VIC_IT); /* select EINT as IRQ source */
//	VICIntEnable = VIC_BIT(SCA3000_DRDY_VIC_IT); /* enable it                 */
//	_VIC_CNTL( MICROMAG_DRDY_VIC_SLOT) = VIC_ENABLE | SCA3000_DRDY_VIC_IT;
//	_VIC_ADDR( MICROMAG_DRDY_VIC_SLOT) = (unsigned int) EXTINT_ISR; // address of the ISR
}



void sca3000_read_res(void) {
    sca3000_status = SCA3000_READING_RES; //set status
    sca3000_select(); //set slave select pin to ground
    SpiClearRti(); //clear spi receive timeout interrupt
    SpiEnableRti(); //enable spi receive timeout interrupt
    /* trigger 2 bytes read */
    unsigned char cmd=0x05<<2;
    SSPDR = cmd;
    SSPDR = 0;
    SSPDR = 0;
    SSPDR = 0;
    SSPDR = 0;
    SSPDR = 0;
    SSPDR = 0;
    SpiEnable(); //enable spi
}

//void sca3000_send_conf(void) {
//	comm_send_string("SCA3000_read_res: ");
//	comm_send_int(command);
//	comm_send_string("\r\n");
//    sca3000_status = SCA3000_SEND_CONF; //set status
//    sca3000_select(); //set slave select pin to ground
//    SpiClearRti(); //clear spi receive timeout interrupt
//    SpiEnableRti(); //enable spi receive timeout interrupt
//    SSPDR = command;
//    SSPDR = 0;
//    SpiEnable(); //enable spi
//}

/*void sca3000_send_req(int axis) {
	sca3000_select(); //set slave select pin to ground
	sca3000_status = SCA3000_SENDING_REQ; //set status
	comm_send_string("MmSendReq for axis: ");
	comm_send_int(axis);
	comm_send_string("\r\n");
	unsigned char control_byte = (axis) << 0 | 4 << 4; //make control byte to read current axis (ratio=/256)
	SSPDR = control_byte; //write control byte into send fifo
	SpiClearRti(); //clear the receive timeout interrupt
	SpiEnableRti(); //enable the receive timeout interrupt
	SpiEnable(); //enable spi and send control byte
}*/


void sca3000_on_spi_int(void) {

			/* read dummy control byte reply */
			unsigned char foo __attribute__ ((unused)) = SSPDR;
			signed char msb;
			msb = SSPDR;
			unsigned char lsb = SSPDR;
			signed int x_acc = msb*32 + lsb/8;
			comm_send_int(x_acc);
			comm_send_string(",");

			msb = SSPDR;
			lsb = SSPDR;
			signed int z_acc = msb*32 + lsb/8;
			comm_send_int(z_acc);
			comm_send_string(",");


			msb = SSPDR;
			lsb = SSPDR;
			signed int y_acc = msb*32 + lsb/8;
			comm_send_int(y_acc);
			comm_send_string("\r\n");

			SpiClearRti();
			SpiDisableRti();
			SpiDisable();
			sca3000_unselect();
			return;

}

static void SSP_ISR(void) {
	ISR_ENTRY();
	sca3000_on_spi_int();
	VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
	ISR_EXIT();
}

//static void EXTINT_ISR(void) {
//	ISR_ENTRY();
//	comm_send_string("EXTINT_ISR\r\n");
//
//    sca3000_status = SCA3000_GOT_EOC;
//    sca3000_read_res();
//
//	/* clear EINT */
//	EXTINT|=  1<< SCA3000_DRDY_EINT;
//
//	VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
//	ISR_EXIT();
//}
