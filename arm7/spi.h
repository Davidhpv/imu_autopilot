/**
* @file spi.h
*
* @brief functions to use spi
*
* This file contains different functions to use spi interface of
* the arm7. It uses the ssp port and select then the spi function
* of the ssp. The spi functions are optimized for reading out sensor
* data.
**/

#ifndef SPI_H_
#define SPI_H_

#include "LPC21xx.h"

static inline void SpiEnable(void) {
	SSPCR1 |= (1 << SSE);
}
static inline void SpiDisable(void) {
	SSPCR1 &= ~(1 << SSE);
}
static inline void SpiEnableRti(void) {
	SSPIMSC |= (1 << RTIM);
}
static inline void SpiDisableRti(void) {
	SSPIMSC &= ~(1 << RTIM);
}
static inline void SpiClearRti(void) {
	SSPICR |= (1 << RTIC);
}

/*! Constant for selecting the spi 5 bit_mode. @see spi_package */
#define SPI_5_BIT_MODE	0x04 << 0
/*! Constant for selecting the spi 8 bit_mode. @see spi_package */
#define SPI_8_BIT_MODE  0x07 << 0
/*! Constant for selecting the spi 16 bit_mode. @see spi_package */
#define SPI_16_BIT_MODE	0x0F << 0

/**
 * @brief struct of data, mode and some spi functions.
 *
 * If you want to communicate with an spi device, you have first
 * to create such a struct. In this struct there is not only data,
 * but also the slave select functions for your device and the spi
 * interrupt handler for your device.
 * @see spi_transmit
 */
typedef struct {
	/*! An array of max 8 frames. You can put in here 8 or 16 bit
	 * values. You have then to choose the correct bit_mode */
	unsigned short data[8];
	/*! the number of frames in the data array */
	unsigned char length;
	/*! The number of bits for one spi transfer. You can
	 * choose here SPI_8_BIT_MODE or SPI_16_BIT_MODE
	 * @see SPI_8_BIT_MODE
	 * @see SPI_16_BIT_MODE */
	unsigned char bit_mode;
	/*! The slave select function for your spi device. You have to
	 * set the slave select pin of your device to low to be able to
	 * communicate with it. */
	void (*slave_select)(void);
	/*! The slave deselect function for your spi device. You have to
	 * set the slave select pin of your device to high if you don't
	 * communicate with it, if you don't do this, other spi devices
	 * might have problems. */
	void (*slave_unselect)(void);
	/*! After the transmission of the data of your spi package is
	 * finished, an spi receive timeout interrupt will occur. The
	 * SPI_ISR will call this interrupt handler. In this interrupt
	 * handler you have to read out dummy frames for data you have
	 * sent or you have to read out the data which you have ordered */
	void (*spi_interrupt_handler)(void);
} spi_package;

/**
 * @brief spi initialization
 *
 * Initialize the spi bus. If you want to use any spi device, you first
 * have to use this initialization function.
 */
void spi_init(void);

/**
 * @brief send and receive data over spi
 *
 * Transmit a spi package over the spi bus. If you want to receive something
 * over spi, you have to send 0 over spi and handle it with your
 * spi_interrupt_handler. Also if you want to just transmit a file you
 * have to read out a dummy frame out of the spi buffer.
 * @param package The package which you want to send over spi. In the package
 * is also the spi_interrupt_handler and the slave select and unselect functions.
 * @see spi_package
 */
void spi_transmit( spi_package* package );

int spi_running(void);

/**
 * With this function you can check how many packages are in the spi buffer.
 * @return number of packages in the spi buffer
 */
int spi_number_of_packages_in_buffer(void);
#endif /* SPI_H_ */
