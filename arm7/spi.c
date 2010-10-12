/*
 * spi.c
 *
 *  Created on: 15.05.2009
 *      Author: Martin Rutschmann
 */

#include "spi.h"
#include "armVIC.h"
#include "LPC21xx.h"
#include "conf.h"
#include "led.h"

#include <stdio.h>

#ifdef SPI_USE_POLLING

void spi_transmit(spi_package* package) {
	/*copy data into fifo*/
	for(int i=0; i<(*package).length; i++){
		SSPDR = (*package).data[i];
	}
	/*set bit mode (8bit or 16bit)*/
	SSPCR0 = (*package).bit_mode |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
	/*select the device*/
	(*package).slave_select();
	/*enable spi and send fifo*/
	SpiEnable();
	/*wait until all data has been sent(Transmit fifo is emtpy)*/
	while(SSPSR&(1<<BSY));
	/*unselect device*/
	(*package).slave_unselect();
	/*disable spi*/
	SpiDisable();
	/*read out the dummy bytes or the data bytes*/
	(*package).spi_interrupt_handler();
}

void spi_init(void){
	/* setup pins for SSP (SCK, MISO, MOSI) */
	PINSEL1 |= SSP_PINSEL1_SCK  | SSP_PINSEL1_MISO | SSP_PINSEL1_MOSI;

	/* setup SSP */
	SSPCR0 = SSPCR0_VAL;
	SSPCR1 = SSPCR1_VAL;
	SSPCPSR = 0x02;
}

int spi_running(void){
	return 0;
}

int spi_number_of_packages_in_buffer(void){
	//in polling mode the buffer isn't needed
	return 0;
}

#else

static void SSP_ISR(void) __attribute__((naked));
static inline void spi_transmit_single_package(spi_package* package);

spi_package spi_package_buffer[SPI_PACKAGE_BUFFER_SIZE];
int spi_package_buffer_insert_idx, spi_package_buffer_extract_idx;
int spi_transmit_running;

spi_package* spi_current_package;

void spi_transmit(spi_package* package) {
	int temp;
	unsigned cpsr;

	temp = (spi_package_buffer_insert_idx + 1) % SPI_PACKAGE_BUFFER_SIZE; // calculate the next queue position

	if (temp == spi_package_buffer_extract_idx) { // check if there is free space in the send queue
		return; // no room
	}

	cpsr = disableIRQ(); // disable global interrupts
	SpiDisableRti(); // disable RTI interrupts
	restoreIRQ(cpsr); // restore global interrupts

	spi_package_buffer[spi_package_buffer_insert_idx] = *package; // add data to queue
	spi_package_buffer_insert_idx = temp; // increase insert pointer

	if (spi_transmit_running==0) // check if in process of sending data
	{
		spi_transmit_running = 1; // set running flag
		spi_transmit_single_package(&spi_package_buffer[spi_package_buffer_extract_idx]);
		spi_package_buffer_extract_idx++;
		spi_package_buffer_extract_idx %= SPI_PACKAGE_BUFFER_SIZE;
	}

	cpsr = disableIRQ(); // disable global interrupts
	SpiEnableRti(); // enable RTI interrupts
	restoreIRQ(cpsr); // restore global interrupts
}

void spi_init(void){
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

	spi_transmit_running = 0;
	spi_package_buffer_insert_idx = 0;
	spi_package_buffer_extract_idx = 0;
}

static inline void spi_transmit_single_package(spi_package* package){
	spi_current_package = package;
	/*copy data into fifo*/
	for(int i=0; i<(*spi_current_package).length; i++){
		SSPDR = (*spi_current_package).data[i];
	}
	/*set bit mode (8bit or 16bit)*/
	SSPCR0 = (*spi_current_package).bit_mode |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
	/*select the device*/
	(*spi_current_package).slave_select();

	/* enable SPI and send package data*/
	SpiClearRti();
	SpiEnableRti();
	SpiEnable();
}

int spi_number_of_packages_in_buffer(void){
	return((spi_package_buffer_insert_idx-spi_package_buffer_extract_idx) % SPI_PACKAGE_BUFFER_SIZE);
}

int spi_running(void){
	return spi_transmit_running;
}

static void SSP_ISR(void) {
	ISR_ENTRY();
	//start the device interrupt handler
	(*spi_current_package).spi_interrupt_handler();

	//unselect device, disable spi
	(*spi_current_package).slave_unselect();
	SpiClearRti();
	SpiDisableRti();
	SpiDisable();

	// check if more data to send
	if (spi_package_buffer_insert_idx != spi_package_buffer_extract_idx) {
		spi_transmit_single_package(&spi_package_buffer[spi_package_buffer_extract_idx]);
		spi_package_buffer_extract_idx++;
		spi_package_buffer_extract_idx %= SPI_PACKAGE_BUFFER_SIZE;
	} else {
		spi_transmit_running = 0; // clear running flag
	}
	VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
	ISR_EXIT();
}

#endif

