/*
 * mainfunctions.h
 *
 *  Created on: 08.10.2010
 *      Author: mackayl
 */

#ifndef MAINFUNCTIONS_H_
#define MAINFUNCTIONS_H_

#include "inttypes.h"
#include "mav_vect.h"

typedef enum
{
	COUNTER1 = 0,
			COUNTER2,
			COUNTER3,
			COUNTER4,
			COUNTER5,
			COUNTER6,
			COUNTER7,
			COUNTER8,
			COUNTER9,
			COUNTER10,
			COUNTER11,
			COUNTER12,
			COUNTER13,
			COUNTER14,
			NUM_OF_COUNTERS
} counter_id_t; ///< Software counters for the mainloop

/** @brief Handle the camera shutter */
void camera_shutter_handling(uint64_t loop_start_time);

/** @brief Fuse position data */
void fuse_vision_altitude_200hz(void);
/** @brief Send the system state to the GCS */
void send_system_state(void);

void adc_read(void);

void communication_send_controller_feedback(void);
void communication_send_attitude_position(uint64_t loop_start_time);
void communication_send_raw_data(uint64_t loop_start_time);
/** @brief Send remote control values to GCS on request */
void communication_send_remote_control(void);
void handle_controller_timeouts(uint64_t loop_start_time);

/** @brief Convert float parameters to faster uint ones */
void sync_state_parameters(void);
/** @brief Update the system state machine */
void update_system_statemachine(uint64_t loop_start_time);

uint8_t handle_reset_request(void);

void handle_eeprom_write_request(void);
/** @brief Convert normalized RC channel values to 0-255 */
uint8_t rc_to_255(int chan);


/** @brief Measures the peak CPU load */
uint16_t measure_peak_cpu_load(uint64_t loop_start_time, uint64_t loop_stop_time, uint64_t min_mainloop);
uint16_t measure_avg_cpu_load(uint64_t loop_start_time,	uint64_t loop_stop_time, uint64_t min_mainloop);
void us_run_init(void);
/** @brief Software timer function for use in the mainloop */
uint8_t us_run_every(uint32_t us, counter_id_t counter_id, uint32_t current_time);

void position_integrate(float_vect3* att,float_vect3 *pos,float_vect3 *vel,float_vect3 *acc);




#endif /* MAINFUNCTIONS_H_ */
