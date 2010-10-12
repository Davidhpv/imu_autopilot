/*
 * mainloop.h
 *
 *  Created on: 08.10.2010
 *      Author: mackayl
 */

#ifndef MAINLOOP_QUADROTOR_H_
#define MAINLOOP_QUADROTOR_H_

/** @brief Run the system */
void main_loop_quadrotor(void);

void main_init_quadrotor(void);

/** @brief Move the controller setpoint interpolation */
void update_controller_setpoints(void);
/** @brief Update controller gains */
void update_controller_parameters(void);


#endif /* MAINLOOP_QUADROTOR_H_ */
