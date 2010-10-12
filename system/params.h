/*
 * params.h
 *
 *  Created on: 23.06.2010
 *      Author: Laurens Mackay
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include "inttypes.h"

void param_write(uint32_t param_id);
void param_start_read(uint32_t param_id);
void param_read_update(uint32_t param_id);

void param_handler(void);

void param_write_all(void);

void param_read_all(void);

//return 1 if number of parameter has not changed else 0
//function will block for some ms
//TODO make sure that this works also if we go up to a parameter we already had once before
//not working at the moment
uint8_t param_size_check(void);



#endif /* PARAMS_H_ */
