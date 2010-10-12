#include "control_fixed_wing_attitude.h"
#include "global_data.h"
#include "radio_control.h"
#include "servos.h"

void control_fixed_wing_attitude_init(void)
{

}

void control_fixed_wing_attitude(void)
{
	float roll_remote = radio_control_get_channel(global_data.param[PARAM_PPM_ROLL_CHANNEL]);
	static float lp_roll = 0;
	lp_roll = 0.8f * lp_roll + 0.2f * -global_data.attitude.x;
	servos_set(0, lp_roll + 0.3f * roll_remote);

	static float lp_pitch = 0;
	lp_pitch = 0.8f * lp_pitch + 0.2f * -global_data.attitude.y;
	float pitch_remote = radio_control_get_channel(global_data.param[PARAM_PPM_NICK_CHANNEL]);
	servos_set(1, lp_pitch + 0.3f * pitch_remote);

	for (int i = 0; i < PWM_NB_CHANNELS; i++)
	{

	}
}
