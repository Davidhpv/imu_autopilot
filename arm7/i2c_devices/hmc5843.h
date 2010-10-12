/*
 * hmc5843.h
 *
 *  Created on: 13.07.2010
 *      Author: mackayl
 */

#ifndef HMC5843_H_
#define HMC5843_H_

#include "i2c.h"
#include "global_data.h"

void hmc5843_init(void);

void hmc5843_start_read(void);

void hmc5843_read_handler(i2c_package *package);

void hmc5843_get_data(int16_vect3* value);

int8_t hmc5843_data_ok(void);

int16_t twos_complement_decode16(int16_t twos_complement);

#endif /* HMC5843_H_ */
