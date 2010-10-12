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
 *   @brief Microchip 24FC256 EEPROM driver
 *
 *   This file contains the device driver for the Microchip 24FC256 EEPROM.
 *   @author Christian Dobler <mavteam@student.ethz.ch>
 *
 */
#ifndef EEPROM_H_
#define EEPROM_H_

#include "inttypes.h"
#include "conf.h"
#include "i2c.h"


#if (FEATURE_EEPROM==FEATURE_EEPROM_ENABLED)
/**
 * @brief Write data to specific address in EEPROM
 *
 * This function writes the "data" byte array to the specified "address"
 * in the EEPROM.
 *
 * @param address 	16 byte start address of the location in the EEPROM, where the data should be written
 * @param length	number of bytes in the "data" byte array (max. 64)
 * @param data		pointer to the byte array, that will be written to the EEPROM
 */
void eeprom_write(uint16_t address, uint8_t length, uint8_t * data);

/**
 * @brief Start data read from specific address in EEPROM
 *
 * This function starts a read-out from the specified address in the EEPROM.
 * This function does not return any data. "eeprom_read_data()" needs to be
 * called after invoking this function to collect the data read out from the
 * EEPROM. Since it will take a few microseconds to read out the data from
 * EEPROM, other tasks can be executed before collecting the read results,
 * but no other EEPROM requests should be started during this period.
 *
 * @param address 	16 byte start address of the location in the EEPROM to read from
 * @param length	number of bytes to be read from the EEPROM (max. MAX_I2C_PACKAGE_SIZE)
 */
void eeprom_start_read(uint16_t address, uint8_t length);

/**
 * @brief Collect read out data
 *
 * This function collects the data read out from the EEPROM by "eeprom_start_read()"
 * in the byte array "data". It has to be called after "eeprom_start_read()" and
 * waits for the completion of the EEPROM read by the I2C subsystem. This function
 * returns when all the data read from the EEPROM is in the byte array "data".
 *
 * @param data		pointer to the byte array, that the data read from the EEPROM is written to
 */
void eeprom_read_data(uint8_t * data);


//check if we have an eeprom
void eeprom_check_start(void);

void eeprom_check_handler(i2c_package *package);

int8_t eeprom_check_ok(void);

#endif

#endif /* EEPROM_H_ */
