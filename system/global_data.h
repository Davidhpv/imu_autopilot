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
 * @file the main file
 *   @author Martin Rutschmann
 *   @author Dominik Honegger
 *   @author Tobias Naegeli
 *   @author Lorenz Meier
 *   @author Christian Dobler
 *   @author Laurens MacKay
 */

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef _GLOBAL_DATA_H_
#define _GLOBAL_DATA_H_
#include "conf.h"
#include "mav_vect.h"
#include "pid.h"
#include "string.h"
#include "mavlink_types.h"

// TODO Move
#define MOTORS_BLOCKED 0
#define MOTORS_ENABLED 1

enum
{
	PARAM_SYSTEM_ID = 0,
	PARAM_COMPONENT_ID,
	PARAM_SYSTEM_TYPE,
	PARAM_SW_VERSION,
	PARAM_IMU_RESET,

	PARAM_UART0_BAUD,
	PARAM_UART1_BAUD,

	PARAM_MIX_REMOTE_WEIGHT,
	PARAM_MIX_POSITION_WEIGHT,
	PARAM_MIX_POSITION_Z_WEIGHT,
	PARAM_MIX_POSITION_YAW_WEIGHT,
	PARAM_MIX_OFFSET_WEIGHT,
	PARAM_TRIMCHAN,

	PARAM_PID_ATT_P,
	PARAM_PID_ATT_I,
	PARAM_PID_ATT_D,
	PARAM_PID_ATT_LIM,
	PARAM_PID_ATT_AWU,

	PARAM_PID_POS_P,
	PARAM_PID_POS_I,
	PARAM_PID_POS_D,
	PARAM_PID_POS_LIM,
	PARAM_PID_POS_AWU,

	PARAM_PID_POS_Z_P,
	PARAM_PID_POS_Z_I,
	PARAM_PID_POS_Z_D,
	PARAM_PID_POS_Z_LIM,
	PARAM_PID_POS_Z_AWU,

	PARAM_PID_YAWPOS_P,
	PARAM_PID_YAWPOS_I,
	PARAM_PID_YAWPOS_D,
	PARAM_PID_YAWPOS_LIM,
	PARAM_PID_YAWPOS_AWU,

	PARAM_PID_YAWSPEED_P,
	PARAM_PID_YAWSPEED_I,
	PARAM_PID_YAWSPEED_D,
	PARAM_PID_YAWSPEED_LIM,
	PARAM_PID_YAWSPEED_AWU,


	PARAM_POSITIONSETPOINT_ACCEPT,
	PARAM_POSITION_TIMEOUT,
	PARAM_POSITION_SETPOINT_X,
	PARAM_POSITION_SETPOINT_Y,
	PARAM_POSITION_SETPOINT_Z,
	PARAM_POSITION_SETPOINT_YAW,
	PARAM_POSITION_YAW_TRACKING,

	PARAM_ATT_OFFSET_X,
	PARAM_ATT_OFFSET_Y,
	PARAM_ATT_OFFSET_Z,

	PARAM_VEL_OFFSET_X,
	PARAM_VEL_OFFSET_Y,
	PARAM_VEL_OFFSET_Z,

	PARAM_VEL_DAMP,
	PARAM_ATT_KAL_KACC,
	PARAM_ATT_KAL_IYAW,

	PARAM_GYRO_OFFSET_X,
	PARAM_GYRO_OFFSET_Y,
	PARAM_GYRO_OFFSET_Z,
	PARAM_CAL_TEMP,
	PARAM_CAL_GYRO_TEMP_FIT_X,
	PARAM_CAL_GYRO_TEMP_FIT_Y,
	PARAM_CAL_GYRO_TEMP_FIT_Z,
	PARAM_CAL_GYRO_TEMP_FIT_ACTIVE,

	PARAM_CAL_MAG_OFFSET_X,
	PARAM_CAL_MAG_OFFSET_Y,
	PARAM_CAL_MAG_OFFSET_Z,

	PARAM_CAL_PRES_DIFF_OFFSET,

	PARAM_ACC_OFFSET_X,
	PARAM_ACC_OFFSET_Y,
	PARAM_ACC_OFFSET_Z,

	PARAM_ACC_NAVI_OFFSET_X,
	PARAM_ACC_NAVI_OFFSET_Y,
	PARAM_ACC_NAVI_OFFSET_Z,

	PARAM_VISION_YAWCORRECT,
	PARAM_VISION_ANG_OUTLAYER_TRESHOLD,


	PARAM_SEND_DEBUGCHAN,

	PARAM_SEND_SLOT_ATTITUDE,
	PARAM_SEND_SLOT_RAW_IMU,
	PARAM_SEND_SLOT_REMOTE_CONTROL,
	PARAM_SEND_SLOT_CONTROLLER_OUTPUT,

	PARAM_SEND_SLOT_DEBUG_1,
	PARAM_SEND_SLOT_DEBUG_2,
	PARAM_SEND_SLOT_DEBUG_3,
	PARAM_SEND_SLOT_DEBUG_4,
	PARAM_SEND_SLOT_DEBUG_5,
	PARAM_SEND_SLOT_DEBUG_6,

	PARAM_PPM_SAFETY_SWITCH_CHANNEL,
	PARAM_PPM_TUNE1_CHANNEL,
	PARAM_PPM_TUNE2_CHANNEL,
	PARAM_PPM_TUNE3_CHANNEL,
	PARAM_PPM_TUNE4_CHANNEL,
	PARAM_PPM_THROTTLE_CHANNEL,
	PARAM_PPM_YAW_CHANNEL,
	PARAM_PPM_ROLL_CHANNEL,
	PARAM_PPM_NICK_CHANNEL,

	PARAM_GPS_MODE,
	PARAM_CAM_INTERVAL,
	PARAM_CAM_EXP,
	ONBOARD_PARAM_COUNT
///< Store parameters in EEPROM and expose them over MAVLink paramter interface
} global_param_id;

// Testing new approach to save 1500 Bytes of RAM if we have 100 parameters.
// can we let the global_data.param_name pointer point to this array in flash?
static const char param_name[ONBOARD_PARAM_COUNT][ONBOARD_PARAM_NAME_LENGTH] =
{
	[PARAM_SYSTEM_ID] = "SYS_ID",
	[PARAM_COMPONENT_ID] = "SYS_COMP_ID",
	[PARAM_SYSTEM_TYPE] = "SYS_TYPE",
	[PARAM_SW_VERSION] = "SYS_SW_VER",
	[PARAM_IMU_RESET] = "SYS_IMU_RESET",
	[PARAM_GPS_MODE] = "GPS_MODE"
};

enum
{
	FLY_WAIT_MOTORS,
	FLY_RAMP_UP,//open loop
	FLY_STARTING,
	FLY_FLYING,
	FLY_SINKING,//4
	FLY_WAIT_LANDING,//5
	FLY_LANDING,//open loop 6
	FLY_RAMP_DOWN,//open loop 7
	FLY_GROUNDED// 8

} state_fly_id;

enum uartmodes
{
	UART_MODE_MAVLINK = 0,
	UART_MODE_GPS = 1,
	UART_MODE_BYTE_FORWARD = 2
};

typedef struct
{
	float_vect3 pos;
	float_vect3 vel;
	float_vect3 ang;

	float confidence;
	uint64_t time_captured;
	uint64_t comp_end;

	int new_data; /* New data available = 1; No new Data = 0 */

} vision_t;

typedef struct
{
	uint8_t vision_ok;//used to switch of position controller in case of vision loss
	uint8_t gps_ok;
	uint8_t ground_distance_ok;
	uint8_t magnet_ok;
	uint8_t pressure_ok;
	uint8_t position_fix;
	uint8_t fly;
	uint8_t attitude_control_enabled;
	uint8_t position_xy_control_enabled;
	uint8_t position_z_control_enabled;
	uint8_t position_yaw_control_enabled;
	//todo use these instead of params
	float mix_remote;
	float mix_position;
	float mix_z_position;
	float mix_yaw_position;
	float mix_offset;
	enum MAV_MODE mav_mode;
	enum MAV_NAV nav_mode;
	enum MAV_TYPE type;
	enum MAV_STATE status;
	uint8_t gps_mode;//< comes from parameter for faster check
	uint8_t uart0mode;
	uint8_t uart1mode;

} sys_state_t;

struct global_struct
{
	uint32_t pressure_raw;                    ///< Raw ambient pressure, in ADC units
	uint32_t pressure_si;					  ///< Pressure in Pascal
	uint32_t pressure_diff_raw;				  ///< Raw differential pressure in ADC units
	uint32_t pressure_diff_si;				  ///< Differential pressure in Pascal
	uint16_vect3 gyros_raw;                   ///< gyros raw sensor data
//	/* gyros raw sensor data offset */
//	uint16_vect3 gyros_raw_offset;
	int16_vect3 accel_raw;                    ///< accel raw sensor data
	int16_vect3 magnet_raw;                   ///< magnetometer raw sensor data
	uint16_t supersonic_raw;                  ///< supersonic raw sensor data
	/* RAW adc data of the gyro temperature sensor */
	uint16_t temperature_gyros;               ///<
	/* temperature sensor data */
	int16_t temperature;                      ///<
	float temperature_si;                     ///< Temperature in degrees celsius

	/// Calibration values and zero offsets
	int16_vect3 gyros_autocal_raw_offset;     ///< Zero-speed offsets of gyros, calibrated at startup

	/// Measurements in physical SI units (http://en.wikipedia.org/wiki/International_System_of_Units)
	///
	float_vect3 gyros_si;                     ///< Angular speed in rad/s
	float_vect3 accel_si;                     ///< Linear acceleration in body frame in m/s^2
	int16_vect3 magnet_corrected;	  		  ///< Magnet Sensor data with corrected offset (raw values)

	/// System state representation
	float_vect3 attitude;                     ///< Angular position / attitude in Tait-Bryan angles (http://en.wikipedia.org/wiki/Yaw,_pitch,_and_roll)
	float_vect3 velocity;                        ///< Current speed of MAV in m/s
	float_vect3 position;                     ///< vector from origin to mav in body coordinates
	float_vect3 position_setpoint;            ///<
	float yaw_pos_setpoint;                   ///<


	float_vect3 position_setpoint_min;        ///<
	float_vect3 position_setpoint_max;        ///<
	float_vect3 position_raw;                 ///<
	float_vect3 position_control_output;      ///< Output of position controller
	float       position_yaw_control_output;
	float_vect3 attitude_control_output;      ///< Output of attitude controller
	float       thrust_control_output;        ///< Output of thrust controller
	float_vect3 attitude_setpoint_pos;        ///< (from position controller)
	float_vect3 attitude_setpoint_pos_body;   ///< In body frame
	float_vect3 attitude_setpoint_pos_body_offset; ///< Additional offset to attitude body control output
	float_vect3 attitude_setpoint_remote;     ///< (from remote)
	float_vect3 attitude_setpoint_offset;     ///< (not aligned IMU to frame, not eliminated sensor offset)
	float_vect3 attitude_setpoint;            ///< angles for the attitude controller

	float gas_remote;
	/*position error in body coordinates*/
	float_vect3 position_error;
	sys_state_t state;                        ///< Current vehicle state representation
	uint8_t motor_block;                      ///< Position of motor block switch
	uint16_t packet_drops;                    ///< Packet drop rate of receiving channels
	uint16_t pwm_values[PWM_NB_CHANNELS];     ///< Servo outputs
	uint16_t ppm_values[PPM_NB_CHANNEL];      ///< RC inputs
	uint32_t watchdog_error;
	uint16_t battery_voltage;                 ///< Battery voltage in mV
	uint16_t cpu_usage;                       ///< CPU usage, 0 = 0%, 1000 = 100%
	uint16_t cpu_peak;                       ///< CPU peak, 0 = 0%, 1000 = 100%
	/* PID state variables */
	PID_t pid_fx, pid_fy, pid_fz, pid_yaw, pid_yawspeed;
	/* Global position setpoint*/
	float_vect3 reference_world;
	float param[ONBOARD_PARAM_COUNT];         ///< EEPROM parameter values
	char param_name[ONBOARD_PARAM_COUNT][ONBOARD_PARAM_NAME_LENGTH];  ///< EEPROM parameter names
	float ground_distance;
	float ground_distance_unfiltered;
	vision_t vision_data;                     ///< Data from computer vision system
	uint64_t pos_last_valid;
	uint64_t entry_critical;

	uint16_t i2c0_err_count;                  ///< I2C0 errors
	uint16_t i2c1_err_count;                  ///< I2C1 errors
	uint16_t spi_err_count;                   ///< SPI errors

	float thrust_hover_offset;
	float thrust_calibration_increment;
	float thrust_landing;
	bool waiting_over;
	bool ramp_up;
	float motor_thrust_actual;

	uint8_t rc_rssi;							  ///< Receive Signal Strength Indicator (0: 0%, 255: 100%)
	uint16_t rc_chan_cal_min[9];
	uint16_t rc_chan_cal_center[9];
	uint16_t rc_chan_cal_max[9];

};

struct global_struct global_data;


/**
 * @brief reset all parameters to default
 * @warning DO NOT USE THIS IN FLIGHT!
 */
static inline void global_data_reset_param_defaults(void){

	global_data.param[PARAM_SYSTEM_ID] = 42;
	strcpy(global_data.param_name[PARAM_SYSTEM_ID], "SYS_ID");

	global_data.param[PARAM_COMPONENT_ID] = 200;
	strcpy(global_data.param_name[PARAM_COMPONENT_ID], "SYS_COMP_ID");

	global_data.param[PARAM_SYSTEM_TYPE] = MAV_GENERIC;
	strcpy(global_data.param_name[PARAM_SYSTEM_TYPE], "SYS_TYPE");

	global_data.param[PARAM_SW_VERSION] = 2000;
	strcpy(global_data.param_name[PARAM_SW_VERSION], "SYS_SW_VER");

	global_data.param[PARAM_PPM_SAFETY_SWITCH_CHANNEL] = 5;
	strcpy(global_data.param_name[PARAM_PPM_SAFETY_SWITCH_CHANNEL],
			"RC_SAFETY_CHAN");

	global_data.param[PARAM_UART0_BAUD] = 115200;// 115200;
	strcpy(global_data.param_name[PARAM_UART0_BAUD], "UART_0_BAUD");

	global_data.param[PARAM_UART1_BAUD] = 57600;//57600
	strcpy(global_data.param_name[PARAM_UART1_BAUD], "UART_1_BAUD");

	global_data.param[PARAM_PPM_TUNE1_CHANNEL] = 7;
	strcpy(global_data.param_name[PARAM_PPM_TUNE1_CHANNEL], "RC_TUNE_CHAN1");
	global_data.param[PARAM_PPM_TUNE2_CHANNEL] = 5;
	strcpy(global_data.param_name[PARAM_PPM_TUNE2_CHANNEL], "RC_TUNE_CHAN2");
	global_data.param[PARAM_PPM_TUNE3_CHANNEL] = 6;
	strcpy(global_data.param_name[PARAM_PPM_TUNE3_CHANNEL], "RC_TUNE_CHAN3");
	global_data.param[PARAM_PPM_TUNE4_CHANNEL] = 8;
	strcpy(global_data.param_name[PARAM_PPM_TUNE4_CHANNEL], "RC_TUNE_CHAN4");

	global_data.param[PARAM_PPM_THROTTLE_CHANNEL] = 3;
	strcpy(global_data.param_name[PARAM_PPM_THROTTLE_CHANNEL], "RC_THRUST_CHAN");

	global_data.param[PARAM_PPM_YAW_CHANNEL] = 4;
	strcpy(global_data.param_name[PARAM_PPM_YAW_CHANNEL], "RC_YAW_CHAN");

	global_data.param[PARAM_PPM_ROLL_CHANNEL] = 2;
	strcpy(global_data.param_name[PARAM_PPM_ROLL_CHANNEL], "RC_ROLL_CHAN");

	global_data.param[PARAM_TRIMCHAN] = 0;
	strcpy(global_data.param_name[PARAM_TRIMCHAN], "RC_TRIM_CHAN");

	global_data.param[PARAM_PPM_NICK_CHANNEL] = 1;
	strcpy(global_data.param_name[PARAM_PPM_NICK_CHANNEL], "RC_NICK_CHAN");

	global_data.param[PARAM_CAM_INTERVAL] = 36000;//32000;
	strcpy(global_data.param_name[PARAM_CAM_INTERVAL], "CAM_INTERVAL");
	global_data.param[PARAM_CAM_EXP] = 1000;
	strcpy(global_data.param_name[PARAM_CAM_EXP], "CAM_EXP");

	global_data.param[PARAM_POSITIONSETPOINT_ACCEPT] = 0;
	strcpy(global_data.param_name[PARAM_POSITIONSETPOINT_ACCEPT],
			"POS_SP_ACCEPT");

	global_data.param[PARAM_POSITION_TIMEOUT] = 2000000;
	strcpy(global_data.param_name[PARAM_POSITION_TIMEOUT],
				"POS_TIMEOUT");



	global_data.param[PARAM_POSITION_SETPOINT_X] =1.1f;//0.21;// 0.5;
	global_data.param[PARAM_POSITION_SETPOINT_Y] =1.1f;//0.145;// 0.44;
	global_data.param[PARAM_POSITION_SETPOINT_Z] = -0.8f;//-1.0;
	global_data.param[PARAM_POSITION_SETPOINT_YAW] = 0.0f;

	global_data.param[PARAM_POSITION_YAW_TRACKING]=0;

	global_data.attitude_setpoint_pos_body_offset.x = 0.0f;
	global_data.attitude_setpoint_pos_body_offset.y = 0.0f;
	global_data.attitude_setpoint_pos_body_offset.z = 0.0f;



	strcpy(global_data.param_name[PARAM_POSITION_SETPOINT_X],
			"POS_SP_X");
	strcpy(global_data.param_name[PARAM_POSITION_SETPOINT_Y],
			"POS_SP_Y");
	strcpy(global_data.param_name[PARAM_POSITION_SETPOINT_Z],
			"POS_SP_Z");
	strcpy(global_data.param_name[PARAM_POSITION_SETPOINT_YAW],
			"POS_SP_YAW");
	strcpy(global_data.param_name[PARAM_POSITION_YAW_TRACKING],
			"POS_YAW_TRACK");


//	// FOR FLYING WITHOUT CABLE
//		global_data.param[PARAM_PID_ATT_P] = 45.3;
//		strcpy(global_data.param_name[PARAM_PID_ATT_P], "PID_ATT_P");
//		global_data.param[PARAM_PID_ATT_I] = 0;//0
//		strcpy(global_data.param_name[PARAM_PID_ATT_I], "PID_ATT_I");
//		global_data.param[PARAM_PID_ATT_D] = 14.9;
//		strcpy(global_data.param_name[PARAM_PID_ATT_D], "PID_ATT_D");
//		global_data.param[PARAM_PID_ATT_AWU] = 1;//1
//		strcpy(global_data.param_name[PARAM_PID_ATT_AWU], "PID_ATT_AWU");

// FOR FLYING WITH CABLE
	global_data.param[PARAM_PID_ATT_P] = 90; // 45 Bravo
	strcpy(global_data.param_name[PARAM_PID_ATT_P], "PID_ATT_P");
	global_data.param[PARAM_PID_ATT_I] = 60; // 15 Bravo
	strcpy(global_data.param_name[PARAM_PID_ATT_I], "PID_ATT_I");
	global_data.param[PARAM_PID_ATT_D] = 30; // 15 Bravo
	strcpy(global_data.param_name[PARAM_PID_ATT_D], "PID_ATT_D");
	global_data.param[PARAM_PID_ATT_LIM] = 100;//not yet used!!!!
	strcpy(global_data.param_name[PARAM_PID_ATT_LIM], "PID_ATT_LIM");
	global_data.param[PARAM_PID_ATT_AWU] = 0.3;//1
	strcpy(global_data.param_name[PARAM_PID_ATT_AWU], "PID_ATT_AWU");

	global_data.param[PARAM_PID_YAWPOS_P] = 5;//1;
	strcpy(global_data.param_name[PARAM_PID_YAWPOS_P], "PID_YAWPOS_P");
	global_data.param[PARAM_PID_YAWPOS_I] = 0.1;//0
	strcpy(global_data.param_name[PARAM_PID_YAWPOS_I], "PID_YAWPOS_I");
	global_data.param[PARAM_PID_YAWPOS_D] = 1;//0.5;
	strcpy(global_data.param_name[PARAM_PID_YAWPOS_D], "PID_YAWPOS_D");
	global_data.param[PARAM_PID_YAWPOS_LIM] = 3;
	strcpy(global_data.param_name[PARAM_PID_YAWPOS_LIM], "PID_YAWPOS_LIM");
	global_data.param[PARAM_PID_YAWPOS_AWU] = 1;//1
	strcpy(global_data.param_name[PARAM_PID_YAWPOS_AWU], "PID_YAWPOS_AWU");

	global_data.param[PARAM_PID_YAWSPEED_P] = 15;
	strcpy(global_data.param_name[PARAM_PID_YAWSPEED_P], "PID_YAWSPEED_P");
	global_data.param[PARAM_PID_YAWSPEED_I] = 5;//0
	strcpy(global_data.param_name[PARAM_PID_YAWSPEED_I], "PID_YAWSPEED_I");
	global_data.param[PARAM_PID_YAWSPEED_D] = 0;
	strcpy(global_data.param_name[PARAM_PID_YAWSPEED_D], "PID_YAWSPEED_D");
	global_data.param[PARAM_PID_YAWSPEED_LIM] = 50;//not yet used!!!!
	strcpy(global_data.param_name[PARAM_PID_YAWSPEED_LIM], "PID_YAWSPE_LIM");
	global_data.param[PARAM_PID_YAWSPEED_AWU] = 1;//1
	strcpy(global_data.param_name[PARAM_PID_YAWSPEED_AWU], "PID_YAWSPE_AWU");

	// Position
	global_data.param[PARAM_PID_POS_P] =1.8;// 1.6f;  //0.5; //2.4;//5
	strcpy(global_data.param_name[PARAM_PID_POS_P], "PID_POS_P");
	global_data.param[PARAM_PID_POS_I] =0.2;// 0.35f; //0.3;//0.1
	strcpy(global_data.param_name[PARAM_PID_POS_I], "PID_POS_I");
	global_data.param[PARAM_PID_POS_D] = 2.0f;  //0.5;//1.6;//1
	strcpy(global_data.param_name[PARAM_PID_POS_D], "PID_POS_D");
	global_data.param[PARAM_PID_POS_LIM] = 0.2;
	strcpy(global_data.param_name[PARAM_PID_POS_LIM], "PID_POS_LIM");
	global_data.param[PARAM_PID_POS_AWU] = 5;//1
	strcpy(global_data.param_name[PARAM_PID_POS_AWU], "PID_POS_AWU");

	global_data.param[PARAM_PID_POS_Z_P] = 0.5;
	strcpy(global_data.param_name[PARAM_PID_POS_Z_P], "PID_POS_Z_P");
	global_data.param[PARAM_PID_POS_Z_I] = 0.3;//0
	strcpy(global_data.param_name[PARAM_PID_POS_Z_I], "PID_POS_Z_I");
	global_data.param[PARAM_PID_POS_Z_D] = 0.2;
	strcpy(global_data.param_name[PARAM_PID_POS_Z_D], "PID_POS_Z_D");
	global_data.param[PARAM_PID_POS_Z_LIM] = 0.30;//1
	strcpy(global_data.param_name[PARAM_PID_POS_Z_LIM], "PID_POS_Z_LIM");
	global_data.param[PARAM_PID_POS_Z_AWU] = 3;//1
	strcpy(global_data.param_name[PARAM_PID_POS_Z_AWU], "PID_POS_Z_AWU");


	global_data.param[PARAM_GYRO_OFFSET_X] = 29760;//29777;//80
	strcpy(global_data.param_name[PARAM_GYRO_OFFSET_X], "CAL_GYRO_X");
	global_data.param[PARAM_GYRO_OFFSET_Y] = 29860;//29835;//29849;
	strcpy(global_data.param_name[PARAM_GYRO_OFFSET_Y], "CAL_GYRO_Y");
	global_data.param[PARAM_GYRO_OFFSET_Z] = 29877;//29880;//29898;
	strcpy(global_data.param_name[PARAM_GYRO_OFFSET_Z], "CAL_GYRO_Z");
	global_data.param[PARAM_CAL_TEMP] = 32.0f;//29880;//29898;
	strcpy(global_data.param_name[PARAM_CAL_TEMP], "CAL_TEMP");

	/** Magnetometer offset calibration */
	strcpy(global_data.param_name[PARAM_CAL_MAG_OFFSET_X], "CAL_MAG_X");
	global_data.param[PARAM_CAL_MAG_OFFSET_X] = 0;
	strcpy(global_data.param_name[PARAM_CAL_MAG_OFFSET_Y], "CAL_MAG_Y");
	global_data.param[PARAM_CAL_MAG_OFFSET_Y] = 0;
	strcpy(global_data.param_name[PARAM_CAL_MAG_OFFSET_Z], "CAL_MAG_Z");
	global_data.param[PARAM_CAL_MAG_OFFSET_Z] = 0;


	global_data.param[PARAM_CAL_GYRO_TEMP_FIT_X] =  3.149677908834623E+04;
	global_data.param[PARAM_CAL_GYRO_TEMP_FIT_Y] =  2.938336621296374e+04;
	global_data.param[PARAM_CAL_GYRO_TEMP_FIT_Z] = 3.015110968108719e+04;
	global_data.param[PARAM_CAL_GYRO_TEMP_FIT_ACTIVE] = 0;

	strcpy(global_data.param_name[PARAM_CAL_GYRO_TEMP_FIT_X], "CAL_FIT_GYRO_X");
	strcpy(global_data.param_name[PARAM_CAL_GYRO_TEMP_FIT_Y], "CAL_FIT_GYRO_Y");
	strcpy(global_data.param_name[PARAM_CAL_GYRO_TEMP_FIT_Z], "CAL_FIT_GYRO_Z");
	strcpy(global_data.param_name[PARAM_CAL_GYRO_TEMP_FIT_ACTIVE], "CAL_FIT_ACTIVE");

	global_data.param[PARAM_CAL_PRES_DIFF_OFFSET] = 10000;
	strcpy(global_data.param_name[PARAM_CAL_PRES_DIFF_OFFSET], "CAL_PRES_DIFF");

	global_data.param[PARAM_ACC_OFFSET_X] = 0;//-13;
	strcpy(global_data.param_name[PARAM_ACC_OFFSET_X], "CAL_ACC_X");
	global_data.param[PARAM_ACC_OFFSET_Y] = 0;//13;
	strcpy(global_data.param_name[PARAM_ACC_OFFSET_Y], "CAL_ACC_Y");
	global_data.param[PARAM_ACC_OFFSET_Z] = 0;
	strcpy(global_data.param_name[PARAM_ACC_OFFSET_Z], "CAL_ACC_Z");

	global_data.param[PARAM_ACC_NAVI_OFFSET_X] = 0;//-8;
	strcpy(global_data.param_name[PARAM_ACC_NAVI_OFFSET_X], "ACC_NAV_OFFS_X");
	global_data.param[PARAM_ACC_NAVI_OFFSET_Y] = 0;// 12;
	strcpy(global_data.param_name[PARAM_ACC_NAVI_OFFSET_Y], "ACC_NAV_OFFS_Y");
	global_data.param[PARAM_ACC_NAVI_OFFSET_Z] = -1000;
	strcpy(global_data.param_name[PARAM_ACC_NAVI_OFFSET_Z], "ACC_NAV_OFFS_Z");

	global_data.param[PARAM_VEL_OFFSET_X] = 0;//-0.015;
	strcpy(global_data.param_name[PARAM_VEL_OFFSET_X], "VEL_OFFSET_X");
	global_data.param[PARAM_VEL_OFFSET_Y] = 0;//0.011;
	strcpy(global_data.param_name[PARAM_VEL_OFFSET_Y], "VEL_OFFSET_Y");
	global_data.param[PARAM_VEL_OFFSET_Z] = 0;
	strcpy(global_data.param_name[PARAM_VEL_OFFSET_Z], "VEL_OFFSET_Z");

	global_data.param[PARAM_VEL_DAMP] = 0.95;
	strcpy(global_data.param_name[PARAM_VEL_DAMP], "VEL_DAMP");

	global_data.param[PARAM_ATT_KAL_KACC] = 0.0033;
	strcpy(global_data.param_name[PARAM_ATT_KAL_KACC], "ATT_KAL_KACC");

	//We don't have Magnetometer therfore we integrate yaw gyro between vision data
	global_data.param[PARAM_ATT_KAL_IYAW] = 1;
	strcpy(global_data.param_name[PARAM_ATT_KAL_IYAW], "ATT_KAL_IYAW");

	global_data.param[PARAM_ATT_OFFSET_X] =0;// -0.08;//-0.11;
	strcpy(global_data.param_name[PARAM_ATT_OFFSET_X], "ATT_OFFSET_X");
	global_data.param[PARAM_ATT_OFFSET_Y] =0;// -0.080;//0.085;
	strcpy(global_data.param_name[PARAM_ATT_OFFSET_Y], "ATT_OFFSET_Y");
	global_data.param[PARAM_ATT_OFFSET_Z] = -0.080;//-0.08;
	strcpy(global_data.param_name[PARAM_ATT_OFFSET_Z], "ATT_OFFSET_Z");

	global_data.param[PARAM_SEND_DEBUGCHAN] = MAVLINK_COMM_0;
	strcpy(global_data.param_name[PARAM_SEND_DEBUGCHAN], "SEND_DEBUGCHAN");

	// Always send attitude - this is a core system information
	global_data.param[PARAM_SEND_SLOT_ATTITUDE] = 1;
	global_data.param[PARAM_SEND_SLOT_RAW_IMU] = 0;
	global_data.param[PARAM_SEND_SLOT_REMOTE_CONTROL] = 0;
	global_data.param[PARAM_SEND_SLOT_CONTROLLER_OUTPUT] = 0;
	global_data.param[PARAM_SEND_SLOT_DEBUG_1] = 0;
	global_data.param[PARAM_SEND_SLOT_DEBUG_2] = 0;//1
	global_data.param[PARAM_SEND_SLOT_DEBUG_3] = 0;
	global_data.param[PARAM_SEND_SLOT_DEBUG_4] = 0;//1
	global_data.param[PARAM_SEND_SLOT_DEBUG_5] = 0;
	global_data.param[PARAM_SEND_SLOT_DEBUG_6] = 0;
	strcpy(global_data.param_name[PARAM_SEND_SLOT_ATTITUDE], "SLOT_ATTITUDE");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_RAW_IMU], "SLOT_RAW_IMU");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_REMOTE_CONTROL], "SLOT_RC");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_CONTROLLER_OUTPUT], "SLOT_CONTROL");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_DEBUG_1], "DEBUG_1");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_DEBUG_2], "DEBUG_2");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_DEBUG_3], "DEBUG_3");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_DEBUG_4], "DEBUG_4");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_DEBUG_5], "DEBUG_5");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_DEBUG_6], "DEBUG_6");

	global_data.param[PARAM_MIX_REMOTE_WEIGHT] = 1;
	strcpy(global_data.param_name[PARAM_MIX_REMOTE_WEIGHT], "MIX_REMOTE");
	global_data.param[PARAM_MIX_POSITION_WEIGHT] = 0;
	strcpy(global_data.param_name[PARAM_MIX_POSITION_WEIGHT], "MIX_POSITION");
	global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 0;
		strcpy(global_data.param_name[PARAM_MIX_POSITION_Z_WEIGHT], "MIX_Z_POSITION");
	global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 1;
	strcpy(global_data.param_name[PARAM_MIX_POSITION_YAW_WEIGHT], "MIX_POS_YAW");
	global_data.param[PARAM_MIX_OFFSET_WEIGHT] = 1;
	strcpy(global_data.param_name[PARAM_MIX_OFFSET_WEIGHT], "MIX_OFFSET");
	global_data.param[PARAM_VISION_YAWCORRECT] = 0;//now done by multitracker //turn by 180 degree 1.57;//turn by 90 degree
	strcpy(global_data.param_name[PARAM_VISION_YAWCORRECT], "VIS_YAWCORR");

	global_data.param[PARAM_VISION_ANG_OUTLAYER_TRESHOLD] = 0.2;
	strcpy(global_data.param_name[PARAM_VISION_ANG_OUTLAYER_TRESHOLD], "VIS_OUTL_TRESH");

	strcpy(global_data.param_name[PARAM_GPS_MODE], "GPS_MODE");
	global_data.param[PARAM_GPS_MODE] = 0; //0: MAVLINK, 960010: GPS; // 9600 1 0: 9600 baud, mode 1 = U-Blox binary, on UART 0

	global_data.param[PARAM_IMU_RESET] = 0;
	strcpy(global_data.param_name[PARAM_IMU_RESET], "SYS_IMU_RESET");

}
/**
 * @brief resets the global data struct to all-zero values
 * @warning DO NOT USE THIS IN FLIGHT!
 */
static inline void global_data_reset(void)
{
	global_data.state.mav_mode = MAV_MODE_UNINIT;
	global_data.state.status = MAV_STATE_UNINIT;
	global_data.cpu_usage = 0;
	global_data.cpu_peak = 0;
	global_data.rc_rssi = 0;

	global_data.state.vision_ok=0;
	global_data.state.gps_ok=0;
	global_data.state.magnet_ok=0;
	global_data.state.ground_distance_ok=0;
	global_data.state.position_fix=0;
	global_data.ground_distance=0;
	global_data.ground_distance_unfiltered = 0;

	global_data.motor_block = MOTORS_BLOCKED;

	global_data.pos_last_valid = 0; // Make sure there is an overflow in the initial condition

	//DONT CHANGE use PARAM!!
	global_data.attitude_setpoint_offset.x = 0.00;
	global_data.attitude_setpoint_offset.y = 0.00;
	global_data.attitude_setpoint_offset.z = 0;

	global_data.yaw_pos_setpoint=0;

	global_data.position_control_output.x = 0.0f;
	global_data.position_control_output.y = 0.0f;
	global_data.position_control_output.z = 0.0f;

	global_data.position_yaw_control_output = 0.0f;

	//safe corridor
	global_data.position_setpoint_min.x=-20;
	global_data.position_setpoint_min.y=-20;
	global_data.position_setpoint_min.z=-1.2;
	global_data.position_setpoint_max.x=20;
	global_data.position_setpoint_max.y=20;
	global_data.position_setpoint_max.z=0;

	// start landing variables initialization
	global_data.thrust_hover_offset = 0.0f;//0.65f;
	global_data.thrust_calibration_increment = 0.00325;//0.00325;//0.065f;//0.0075
	global_data.thrust_landing=0;
	global_data.motor_thrust_actual=0;
	global_data.temperature_si=0;

	//not used anymore?
	global_data.waiting_over = false;
	global_data.ramp_up = false;
}

#endif /* _GLOBAL_DATA_H_ */

#ifdef __cplusplus
}
#endif
