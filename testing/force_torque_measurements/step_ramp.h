/*
 * single_ramp_upper.h
 *
 *  Created on: 04.05.2009
 *      Author: Administrator
 */

#ifndef SINGLE_RAMP_UPPER_H_
#define SINGLE_RAMP_UPPER_H_

void single_ramp_init(void);

void single_ramp_upper(double t_ramp, double ramp_size);
void single_ramp_lower(double t_ramp, double ramp_size);
void single_step_upper(double step_size);
void single_step_lower(double step_size);

#endif /* SINGLE_RAMP_UPPER_H_ */
