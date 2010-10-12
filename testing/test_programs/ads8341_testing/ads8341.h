/*
 * scp1000.h
 *
 *  Created on: 11.05.2009
 *      Author: Martin Rutschmann
 */

#ifndef ADS8341_H_
#define ADS8341_H_

#include "conf.h"

/* SSPCR0 settings */
#define SSP_DDS  0x04 << 0  /* data size         : 5 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : data captured on second clock transition */
#define SSP_CPHA 0x00 << 7  /* clock phase       : SCK idles low */
#define SSP_SCR  0x0F << 8  /* serial clock rate : divide by 16  */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  /* slave output disable : don't care when master */

#define SSPCR0_VAL (SSP_DDS |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR )
#define SSPCR1_VAL (SSP_LBM |  SSP_SSE | SSP_MS | SSP_SOD )

#define SSP_PINSEL1_SCK  (2<<2)
#define SSP_PINSEL1_MISO (2<<4)
#define SSP_PINSEL1_MOSI (2<<6)

#define SCP1000_STA_STOPPED         0
#define SCP1000_STA_WAIT_EOC        1
#define SCP1000_STA_GOT_EOC         2
#define SCP1000_STA_SENDING_REQUEST 3
#define SCP1000_STA_DATA_AVAILABLE  4
#define SCP1000_STA_READ_STATUSREG  5

#define ADS8341_SS_IODIR IO1DIR
#define ADS8341_SS_IOSET IO1SET
#define ADS8341_SS_IOCLR IO1CLR
#define ADS8341_SS_IOPIN IO1PIN

void ads8341_init(void);
unsigned int ads8341_get_value(int channel);
void ads8341_read(int channel);

#endif /* ADS8341_H_ */
