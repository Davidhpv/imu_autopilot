/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief SCA3100 linear accelerometer device driver
 *
 *   This file contains the device driver for the VTI SCA3100 linear accelerometer.
 *   @author Christian Dobler <mavteam@student.ethz.ch>
 *
 */


#ifndef SCA3100_H_
#define SCA3100_H_

#define SCA3100_MAX_NEG_VALUE 	8192		///< max. neg. value of sensor for two's complement calculation

/**
 * @brief Initialization of the SCA3100 linear accelerometer
 *
 * This function initializes the SCA3100 linear accelerometer and sets the chip
 * select pin into operational mode. The SPI subsystem has to be initialized
 * beforehand; otherwise this will fail.
 */
void sca3100_init(void);

/**
 * @brief Starts reading of current acceleration measurements
 *
 * This function starts the read operation of the SCA3100 sensor values over the
 * SPI bus. The SPI subsystem will complete the data transfer and the measurements
 * are temporarily saved in the SCA3100 driver. Execute "while (spi_running()) ;"
 * to wait for the results. Then call sca3100_get_value() to retrieve
 * the measurements read out of the sensor.
 */
void sca3100_read_res(void);

/**
 * @brief Retrieves most recent measurement from SCA3100 device driver
 *
 * This function retrieves the most recent measurement from SCA3100 device driver.
 * To read out the current values from the sensor, execute sca3100_read_res() first.
 */
int sca3100_get_value(int axis);

/**
 * @brief Resets the PORST bit
 *
 * This function is used to reset the PORST (power-on-reset) bit in the status byte
 * transmitted by the SCA3100. This bit is set when the sensor has been reset and needs
 * to be cleared. Call this function once after reset of the MCU as soon as SPI, interrupts
 * and timers are running. If these components are not running, this function will fail.
 */
void sca3100_reset_porst_bit(void);

/**
 * @brief Resets the PORST bit
 *
 * This function is used to reset the INT-STATUS register to the normal state.
 * Call this function once after reset of the MCU as soon as SPI, interrupts and
 * timers are running. If these components are not running, this function will fail.
 */
void sca3100_read_int_status(void);


#endif /* SCA3100_H_ */
