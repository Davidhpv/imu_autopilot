/*
 * sca3000_testing.h
 *
 *  Created on: May 14, 2009
 *      Author: mavteam
 */


#ifndef SCA3000_H_
#define SCA3000_H_

//SPI SETINGS////////////////////////////////////////////////////////////
/* SSPCR0 settings */
#define SSP_DDS  0x07 << 0  /* data size         : 8 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : data captured on first clock transition */
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

//Slave Select PIN///////////////////////////////////////////////////////
/* Micromag on SSP, IMU connector */
//Slaveselect pin on P0.28
#define SCA3000_SS_PIN   28
#define SCA3000_SS_IODIR IO0DIR
#define SCA3000_SS_IOSET IO0SET
#define SCA3000_SS_IOCLR IO0CLR

//DATAREADY PIN///////////////////////////////////////////////////////////
//Dataready pin on P0.16/EINT0
#define SCA3000_DRDY_PINSEL PINSEL1
#define SCA3000_DRDY_PINSEL_BIT 0
#define SCA3000_DRDY_PINSEL_VAL 1
#define SCA3000_DRDY_EINT 0
#define SCA3000_DRDY_VIC_IT VIC_EINT0


#define sca3000_select() (SCA3000_SS_IOCLR|=1<<SCA3000_SS_PIN)
#define sca3000_unselect() (SCA3000_SS_IOSET|=1<<SCA3000_SS_PIN)

//other functions/////////////////////////////////////////////////////////
void sca3000_init(void);
void sca3000_read_res(void);
//void sca3000_send_req(int axis);


#define SCA3000_IDLE            0
#define SCA3000_BUSY            1
#define SCA3000_SENDING_REQ     2
#define SCA3000_WAITING_EOC     3
#define SCA3000_GOT_EOC         4
#define SCA3000_READING_RES     5
#define SCA3000_DATA_AVAILABLE  6
#define SCA3000_SEND_CONF		7

#endif /* SCA3000_H_ */
