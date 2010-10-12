#include "control_fixed_wing_attitude.h"
#include "global_data.h"
#include "radio_control.h"
#include "servos.h"
#include "pid.h"
#include "altitude_speed.h"
#include "range.h"

PID_t aircraft_speed_pid;

void control_fixed_wing_speed_init(void)
{
	pid_init(&aircraft_speed_pid, 1.0f, 0.0f, 0.0f, 0.0f, 1, 0);
}

/**
 * @brief Control the aircraft speed
 *
 * This function sets the speed according to the speed_setpoint value. It is limited by the remote
 * speed channel, so the thrust can be limited with the remote control as a safety feature.
 * The output is send to servo 2.
 *
 * @return Sets the value of servo 2 to the controller output
 */
void control_fixed_wing_speed(void)
{
	float thrust_remote = (radio_control_get_channel(global_data.param[PARAM_PPM_GAS_CHANNEL])+1.0f)/2.0f;

	float speed_setpoint = 8.0f; // 8 m/s
	float curr_speed = get_indicated_airspeed();
	float curr_acc = global_data.accel_si.x;
	float timediff = 0.05; // 200 Hz, in seconds, should be measured instead of hardcoding

	// Update PID controller
	float output = pid_calculate(&aircraft_speed_pid, speed_setpoint, curr_speed, curr_acc, timediff);

	servos_set(2, min(output, thrust_remote));
}
