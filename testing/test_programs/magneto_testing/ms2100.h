/*
 * micromag_testing.h
 *
 *  Created on: May 14, 2009
 *      Author: mavteam
 */


#ifndef MS2100_H_
#define MS2100_H_

//SPI SETINGS////////////////////////////////////////////////////////////
/* SSPCR0 settings */
#define SSP_DDS  0x07 << 0  /* data size         : 8 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : data captured on first clock transition */
#define SSP_CPHA 0x00 << 7  /* clock phase       : SCK idles low */
#define SSP_SCR  0x08 << 8  /* serial clock rate : divide by 16  */

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
//Slaveselect pin on cam-sw on twog board P1.22
#define MM_SS_PIN   22
#define MM_SS_IODIR IO0DIR
#define MM_SS_IOSET IO0SET
#define MM_SS_IOCLR IO0CLR

//Magnet reset pin////////////////////////////////////////////////////////
//Magnet reset pin on ADC3 on twog board P0.04
#define MM_RESET_PIN   29
#define MM_RESET_IODIR IO0DIR
#define MM_RESET_IOSET IO0SET
#define MM_RESET_IOCLR IO0CLR

//DATAREADY PIN///////////////////////////////////////////////////////////
//Dataready pin on P0.3/EINT1
#define MM_DRDY_PINSEL PINSEL1
#define MM_DRDY_PINSEL_BIT 28
#define MM_DRDY_PINSEL_VAL 2
#define MM_DRDY_EINT (1<<3)
#define MM_DRDY_VIC_IT VIC_EINT3

//RESET FUNCTIONS/////////////////////////////////////////////////////////
#define MmReset() (MM_RESET_IOCLR|=1<<MM_RESET_PIN)//TO LOW
#define MmSet() (MM_RESET_IOSET|=1<<MM_RESET_PIN)//TO HIGH

#define MmSelect() (MM_SS_IOCLR|=1<<MM_SS_PIN)
#define MmUnselect() (MM_SS_IOSET|=1<<MM_SS_PIN)

//other functions/////////////////////////////////////////////////////////
void micromag_init(void);
void MmReadRes(void);
void MmSendReq(int axis);
void MmSendMOT(void);
short new_val;

#define MM_IDLE            0
#define MM_BUSY            1
#define MM_SENDING_REQ     2
#define MM_WAITING_EOC     3
#define MM_GOT_EOC         4
#define MM_READING_RES     5
#define MM_DATA_AVAILABLE  6

#endif /* MS2100_H_ */
