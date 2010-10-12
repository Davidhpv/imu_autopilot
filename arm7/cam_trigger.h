/**
* @file cam_trigger.h
*
* @brief functions to generate a trigger signal for the camera
*
* This file contains the initialization of the camera trigger signal.
* This trigger signal is generated with a pwm output of the arm7. This
* are not the same pwm outputs which are generated with the 4017 decade
* counter for the servos and motor controllers.
*
**/

#ifndef CAM_TRIGGER_H_
#define CAM_TRIGGER_H_

void cam_trigger_init(void);

#endif /* CAM_TRIGGER_H_ */
