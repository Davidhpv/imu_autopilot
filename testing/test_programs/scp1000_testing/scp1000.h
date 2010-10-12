/*
 * scp1000.h
 *
 *  Created on: 11.05.2009
 *      Author: Martin Rutschmann
 */

#ifndef SCP1000_H_
#define SCP1000_H_

/* SSPCR0 settings */
#define SSP_DDS  0x07 << 0  /* data size         : 8 bits        */
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

/* SS on P0.2 */
#define SCP_SS_IODIR IO0DIR
#define SCP_SS_IOSET IO0SET
#define SCP_SS_IOCLR IO0CLR
#define SCP_SS_IOPIN IO0PIN
#define SCP_SS_PIN   20

/* DRDY on P0.3 ( EINT1 ) */
#define SCP_DRDY_IOPIN  	IO0PIN
#define SCP_DRDY_PIN    	3
#define SCP_DRDY_PINSEL 	PINSEL1
#define SCP_DRDY_PINSEL_BIT	0
#define SCP_DRDY_PINSEL_VAL	1
#define SCP_DRDY_EINT		0
#define SCP_DRDY_VIC_IT		VIC_EINT0

/* SCP1000 measurement methods */
#define SCP_MEAS_HIGH_SPEED 0x09
#define SCP_MEAS_HIGH_RES   0x0A
#define SCP_MEAS_TRIG	    0x0C
#define SCP_MEAS_STOP		0x00
#define SCP_MEAS_INIT		0x07
#define SCP_MEAS_SELFTEST	0x0F

void scp1000_init(void);
void scp1000_send_config(unsigned char data);
unsigned int scp1000_get_value(void);
void scp1000_read(void);
int scp1000_get_statusreg(void);
void scp1000_read_statusreg(void);
void scp1000_send_software_reset(void);

#endif /* SCP1000_H_ */
