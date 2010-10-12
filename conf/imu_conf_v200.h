/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Martin Rutschmann <mavteam@student.ethz.ch>
  Dominik Honegger <mavteam@student.ethz.ch>
  Lorenz Meier <lm@student.ethz.ch>
Contributing Authors (in alphabetical order):
  Christian Dobler <mavteam@student.ethz.ch>


Todo:


(c) 2009 PIXHAWK PROJECT

This file is part of the PIXHAWK project

    mavlib is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    mavlib is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with mavlib. If not, see <http://www.gnu.org/licenses/>.

========================================================================*/

/**
* @file imu_conf_v200.h
*
* @brief Settings for the initial revision of the PIXHAWK flight board
*
**/

#ifndef IMU_CONF_V200_H_
#define IMU_CONF_V200_H_

#define FEATURE_CONTROL FEATURE_CONTROL_2010_NONE //disable controll loop
#define FEATURE_BATTERY FEATURE_BATTERY_2010

#define CPU_ARM7

/*! @brief Master oscillator freq.       */
#define FOSC (12000000)
/*! @brief PLL multiplier                */
#define PLL_MUL (5)
/*! @brief CPU clock freq.               */
#define CCLK (FOSC * PLL_MUL)
/*! @brief Peripheral bus speed mask 0x00->4, 0x01-> 1, 0x02 -> 2   */
#define PBSD_BITS 0x00
/*! @brief Peripheral bus clock divider	 */
#define PBSD_VAL 4
/*! @brief Peripheral bus clock freq.    */
#define PCLK (CCLK / PBSD_VAL)


/*======================================================================*/


/** @name LED
 *  Pin settings for the LEDs. */
//@{

#define FEATURE_LED  FEATURE_LED_ENABLED
// The green led is on pin P1.17 (as defined by IO1 port and pin 17)

/*! LED pin. This is the pin on witch the led is connected. */
#define LED_GREEN			17
/*! GPIO direction register for LED (IO0DIR or IO1DIR) */
#define LED_GREEN_DIR		IO1DIR
/*! GPIO clear register for LED (IO0CLR or IO1CLR) */
#define LED_GREEN_CLEAR		IO1CLR
/*! GPIO set register for LED (IO0SET or IO1SET) */
#define LED_GREEN_SET		IO1SET
/*! GPIO pin register for LED (IO0PIN or IO1PIN) */
#define LED_GREEN_PIN		IO1PIN

// The red led is on pin P1.16 (as defined by IO1 port and pin 16)
/*! LED pin. This is the pin on witch the led is connected. */
#define LED_RED			16
/*! GPIO direction register for LED (IO0DIR or IO1DIR) */
#define LED_RED_DIR		IO1DIR
/*! GPIO clear register for LED (IO0CLR or IO1CLR) */
#define LED_RED_CLEAR	IO1CLR
/*! GPIO set register for LED (IO0SET or IO1SET) */
#define LED_RED_SET		IO1SET
/*! GPIO pin register for LED (IO0PIN or IO1PIN) */
#define LED_RED_PIN		IO1PIN
//@}


/*======================================================================*/


/** @name PPM
 *  Settings for recording a ppm input from a remote control */
//@{
// (P0.4) is used as PPM in (CAP0.2)
/*! The settings for P0.6 are on Pinsel0 */
#define PPM_CAPTURE_PINSEL 					PINSEL0
/*! Choose pin 2 (hence CAP0.2) */
#define PPM_CAPTURE_PINSEL_VAL 				2
/*! The 12 and the 13 bit of pinsel 0 are for the P0.6 settings */
#define PPM_CAPTURE_PINSEL_BIT 				12
/*! Timer interrupt number from capture register */
#define PPM_CAPTURE_TIMER_INTERRUPT 		TIR_CR2I
/*! Capture control register (T0CCR if ppm is on timer0, T1CCR if it is on timer1) */
#define PPM_CAPTURE_CONTROL_REGISTER		T0CCR
/*! enable capture 0.2 on falling edge + trigger interrupt */
#define PPM_CAPTURE_CONTROL_REGISTER_VALUE	TCCR_CR2_F | TCCR_CR2_I
/*! capture register with the captured value */
#define PPM_CAPTURE_CAPTURE_REGISTER		T0CR2
//@}

//Slave Select PIN///////////////////////////////////////////////////////
/* Micromag on SSP, IMU connector */
//Slave select pin on P1.22
#define MS2100_SS_PIN   22
#define MS2100_SS_IODIR IO0DIR
#define MS2100_SS_IOSET IO0SET
#define MS2100_SS_IOCLR IO0CLR

//Magnet reset pin////////////////////////////////////////////////////////
//Magnet reset pin on P0.25
#define MS2100_RESET_PIN   25
#define MS2100_RESET_IODIR IO0DIR
#define MS2100_RESET_IOSET IO0SET
#define MS2100_RESET_IOCLR IO0CLR

//DATAREADY PIN///////////////////////////////////////////////////////////
//Dataready pin on P0.30/EINT3
#define MS2100_DRDY_PINSEL PINSEL1
#define MS2100_DRDY_PINSEL_BIT 28
#define MS2100_DRDY_PINSEL_VAL 2
#define MS2100_DRDY_EINT 3
#define MS2100_DRDY_VIC_IT VIC_EINT3

/*======================================================================*/

#define FEATURE_SENSORS	 FEATURE_SENSORS_ENABLED
#define FEATURE_ADC		FEATURE_ADC_PIXHAWK
/** @name SCA3100
 *  Settings for sca3100 sensor */
//@{
#define FEATURE_ACC FEATURE_ACC_3100
#define SCA3100_SS_PIN   18
#define SCA3100_SS_IODIR IO1DIR
#define SCA3100_SS_IOSET IO1SET
#define SCA3100_SS_IOCLR IO1CLR

#define SCA3100_X_AXIS 0
#define SCA3100_Y_AXIS 1
#define SCA3100_Z_AXIS 2
//@}

/*======================================================================*/

/** @name ACCELERATION
 *  Settings for acceleration sensors */
//@{
/*! Neutral output of the adc for the x acceleration */
#define ACC_X_NEUTRAL 32835
/*! Neutral output of the adc for the y acceleration */
#define ACC_Y_NEUTRAL 32760
/*! Neutral output of the adc for the z acceleration */
#define ACC_Z_NEUTRAL 33144
//@}


/*======================================================================*/

/** @name MAGNETOMETER
 *  Settings for magnetometer sensors */
//@{
#define MAG_SENSOR_OFFSET_X	-4
#define MAG_SENSOR_OFFSET_Y	-7
#define MAG_SENSOR_OFFSET_Z	11
#define MAG_MOTOR_OFFSET_X	156
#define MAG_MOTOR_OFFSET_Y	5
#define MAG_MOTOR_OFFSET_Z	-202

#define MAG_RANGE_FACTOR_X (int)(1000*1)
#define MAG_RANGE_FACTOR_Y (int)(1000*73/67)
#define MAG_RANGE_FACTOR_Z (int)(1000*64/112)
//@}

/**
 * @name ANALOG TO DIGITAL CONVERTER (ADC)
 */
//@{
// XY-GYRO (y = roll, x = pitch) ADC
#define ADS8341_0			0
#define ADS8341_0_SS_IODIR	IO1DIR
#define ADS8341_0_SS_IOSET	IO1SET
#define ADS8341_0_SS_IOCLR	IO1CLR
#define ADS8341_0_SS_IOPIN	IO1PIN
// P1.25 (as defined by using IO1DIR/IO1SET and number 25)
// named CS XY-GYRO PRESSURE in schematic
#define ADS8341_0_SS_PIN   	25

//@}
/*======================================================================*/


/** @name GYROS
 *  Settings for gyros sensors */
//@{
/*! Neutral output of the adc for the yaw gyro */
#define GYROS_YAW_NEUTRAL	32349
/*! Neutral output of the adc for the pitch gyro */
#define GYROS_PITCH_NEUTRAL	34253
/*! Neutral output of the adc for the roll gyro */
#define GYROS_ROLL_NEUTRAL	32107

/*! Channel of the max1168 analog input of the roll gyro sensor */
#define GYROS_ROLL_ADS8341_0_CHANNEL	1
/*! Channel of the max1168 analog input of the pitch gyro sensor */
#define GYROS_PITCH_ADS8341_0_CHANNEL	0
/*! Channel of the max1168 analog input of the yaw gyro sensor */
#define GYROS_YAW_ADS8341_0_CHANNEL		2

//@}

/*======================================================================*/


/**
 * @name TEMPERATURE
 *
 * Setting for the temperature readout.
 */
//@{
/*! Digital Analog Converter settings. The DAC pin is fixed to P0.25/AD0.4/AOUT. */
#define TEMPERATURE_ADS8341_0_CHANNEL 	3



//@}

/*======================================================================*/


/** @name BAROMETER
 *  Settings for barometer readout, including DAC. The Digital Analog Converter is used
 *  to adjust dynamically the offset of the barometer, allowing a very high precision. */
//@{
/*! Digital Analog Converter settings. The DAC pin is fixed to P0.25/AD0.4/AOUT. */
/*#define DAC_VALUE_MASK	(0x3FF)
#define DAC_PINSEL		PINSEL1
#define DAC_PINSEL_VAL	2
#define DAC_PINSEL_BIT	18

#define PRESSURE_ADS8341_0_CHANNEL 3
*/
//@}


/*======================================================================*/


/** @name MOTORS
 *  Settings for motors */
//@{
/*! The pwm channel of lower motor  */
//#define MOTORS_LOWER_PWM_CHANNEL 3
/*! The pwm channel of upper motor  */
//#define MOTORS_UPPER_PWM_CHANNEL 2
//@}


/*======================================================================*/


/** @name SERVOS
 *  Settings for servos */
//@{
/*! The pwm channel of servo 1 */
#define SERVOS_SERVO_NEGY_PWM_CHANNEL 5
/*! The pwm channel of servo 2 */
#define SERVOS_SERVO_POSY_PWM_CHANNEL 0
//@}


/*======================================================================*/



/*======================================================================*/


/** @name UART
 *  Settings for the two UART ports of the arm7 */
//@{
/*! Uart 0 receive buffer size (in Bytes) */
#define UART0_RX_BUFFER_SIZE 512
/*! Uart 0 transmit buffer size (in Bytes) */
#define UART0_TX_BUFFER_SIZE 512

/*! Uart 1 receive buffer size (in Bytes) */
#define UART1_RX_BUFFER_SIZE 512
/*! Uart 1 transmit buffer size (in Bytes) */
#define UART1_TX_BUFFER_SIZE 512
//@}


/*======================================================================*/


/** @name PWM
 *  Settings for the pwm generation with the 4017 decade counter */
//@{
/* P0.5 as MAT0.1  */
#define PWM_CLOCK_PIN  5
#define PWM_CLOCK_PINSEL PINSEL0
#define PWM_CLOCK_PINSEL_VAL 0x02
#define PWM_CLOCK_PINSEL_BIT 10
/* p0.20 as GPIO */
#define PWM_RESET_PIN 20
#define PWM_RESET_IODIR IO1DIR
#define PWM_RESET_IOSET IO1SET
#define PWM_RESET_IOCLR IO1CLR
/* Number of used channels */
#define PWM_NB_CHANNELS 9
/* minumum pulse width in usec */
#define PWM_MIN_PULSE_USEC 1000
/* maximum pulse width in usec */
#define PWM_MAX_PULSE_USEC 2000
/* length of the pwm periode in usec */
#define PWM_PERIODE SYS_TICS_OF_USEC(20000)
//@}


/*======================================================================*/


/** @name CAM_TRIGGER
 *  Settings for the cam trigger signal */
//@{
/* P0.21 (IRV on TWOG) as PWM5 output */
//#define CAM_TRIGGER_PIN_PINSEL	 PINSEL1
//#define CAM_TRIGGER_PIN_VAL 		0x01
//#define CAM_TRIGGER_PIN_BIT 		  10

/* We use a PWM5 output, so the matchregister has to be PWMMR5 */
//#define CAM_TRIGGER_PWM_CHANNEL_MR			PWMMR5
/* We use a PWM5 output, this has to be enabled by writing PWMPCR_ENA5 into PWMPCR */
//#define CAM_TRIGGER_PWM_CHANNEL_PCR_ENABLE	PWMPCR_ENA5

/* Period of the cam trigger pulse signal in us */
//#define CAM_TRIGGER_PERIODE_US		100000
/* Pulse width of the cam trigger pulse signal in us */
//#define CAM_TRIGGER_PULSE_WIDTH_US	10000
//@}


/*======================================================================*/
/*==========Following conf not commented yet============================*/
/*======================================================================*/




/* SPI settings *********************************************************/

/* SSPCR0 settings */
#define SSP_DDS  0x0F << 0  /* data size         : 16 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : data captured on first clock transition */
#define SSP_CPHA 0x00 << 7  /* clock phase       : SCK idles low */
#define SSP_SCR  0x0F << 8  /* serial clock rate : The spi clockrate is PCLK/(CPSDVSR * [SSP_SCR+1]) where CPSDVSR is fixed equal to 2 */

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

#define SPI_PACKAGE_BUFFER_SIZE	10

/*  ********************************************************************/

/* I2C settings *********************************************************/
#define FEATURE_I2C					FEATURE_I2C_ENABLED

#define I2C_WRITE					0
#define I2C_READ					1

#define I2C_PACKAGE_BUFFER_SIZE		10
#define MAX_I2C_PACKAGE_SIZE		32
#define LPC_I2C_ADR					0x20
#define I2C_PERMANENT_ERROR_LIMIT	1000

/* I2C0 settings */
/* I2C0 clock settings: I2C_CLOCK = PCLK/(I2C0SCLH_VAL+I2C0SCLL_VAL) = 1MHz */
#define I2C0SCLH_VAL		7  		/* number of PCLK clock cycles that SCL stays high */
#define I2C0SCLL_VAL  		8		/* number of PCLK clock cycles that SCL stays high */
/* I2C0 mode setting */
#define I2C0CONSET_VAL 		0x40  	/* master mode only */

#define I2C0_PINSEL0_SCL  	(1<<4)
#define I2C0_PINSEL0_SDA	(1<<6)


/* I2C1 settings */
/* I2C1 clock settings: I2C_CLOCK = PCLK/(I2C0SCLH_VAL+I2C0SCLL_VAL) = 1MHz */
#define I2C1SCLH_VAL		7  		/* number of PCLK clock cycles that SCL stays high */
#define I2C1SCLL_VAL  		8		/* number of PCLK clock cycles that SCL stays high */
/* I2C1 mode setting */
#define I2C1CONSET_VAL 		0x40  	/* master mode only */

#define I2C1_PINSEL0_SCL 	(3<<22)
#define I2C1_PINSEL0_SDA 	(3<<28)

/*  ********************************************************************/

/**********************************************************/

/** @name PRESSURE **/

#define FEATURE_SENSOR_PRESSURE FEATURE_SENSOR_PRESSURE_BMP085
#define FEATURE_SENSOR_PRESSURE_INTERUPT FEATURE_SENSOR_PRESSURE_INTERUPT_DISABLED

#define BMP085_I2C_BUS_NUMBER				1

#define BMP085_PRESSURE_CONVERSION_MODE		0   //0, 1 or 2

/*  ********************************************************************/

/*  ********************************************************************/

/**********************************************************/

/** @name EEPROM **/

#define EEPROM_I2C_BUS_NUMBER				1

#define EEPROM_I2C_SLAVE_ADDRESS			0xA0	///< I2C slave address of Microchip 24FC256 EEPROM (0b10100000)


/*  ********************************************************************/

/**********************************************************/

/** @name I2C Motor Controllers **/

#define MOT_I2C_BUS_NUMBER					0

#define MOT1_I2C_SLAVE_ADDRESS			0xB2	///< I2C slave address of I2C motor controller number 1
#define MOT2_I2C_SLAVE_ADDRESS			0xB4	///< I2C slave address of I2C motor controller number 2
#define MOT3_I2C_SLAVE_ADDRESS			0xB6	///< I2C slave address of I2C motor controller number 3
#define MOT4_I2C_SLAVE_ADDRESS			0xB8	///< I2C slave address of I2C motor controller number 4


/*  ********************************************************************/

#endif /* IMU_CONF_V200_H_ */
