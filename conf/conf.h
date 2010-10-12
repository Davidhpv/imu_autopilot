/**
* @file
*
* 	@brief General Configuration
*
* 	These settings are general and valid across boards.
*
**/

#ifndef _CONF_H_
#define _CONF_H_

#include "user_conf.h"
#include "stdbool.h"
#include "features.h"


// Select the board
// has to be IMU_PIXHAWK_V200, IMU_PIXHAWK_V210
#define IMU_PIXHAWK_V210 // IMU_PIXHAWK_AMIRANDIIMU_PIXHAWK_V200//

//############### GENERAL SETUP #####################

#define ONBOARD_PARAM_NAME_LENGTH 15    ///< Parameter names are transmitted with max. 15 chars over MAVLink
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define SW_VERSION 2000
#define PPM_SAFETY_SWITCH_CHANNEL 5     ///< The PPM remote control channel where the safety switch is attached to

/* Comm settings *********************************************************/

#define COMM_UART_MODE  UART_8N1

/* Periodic events ***************************************/

#define PERIODIC_TASK_SEC	20e-3
#define PERIODIC_TASK_PERIOD SYS_TICS_OF_SEC( PERIODIC_TASK_SEC )
#define START_TIME_PERIODIC SYS_TICS_OF_USEC( 1000000 )
#define START_TIME_PWM (START_TIME_PERIODIC + SYS_TICS_OF_USEC( 10000 ))

/*! Number of channels which should be received by PPM IN */
#define PPM_NB_CHANNEL 9
/*! The minimum length of the pause of the ppm signal, used to detect failures */
#define PPM_SYNC_MIN_LEN 3000
/*! The maximum length of the pause of the ppm signal, used to detect failures */
#define PPM_SYNC_MAX_LEN 15000
/*! The minimum length of the data of the ppm signal, used to detect failures */
#define PPM_DATA_MIN_LEN 800
/*! The maximum length of the data of the ppm signal, used to detect failures */
#define PPM_DATA_MAX_LEN 2200

/* Comm settings *********************************************************/

/*  ********************************************************************/

#define WATCHDOG_NB_OF_CHANNELS 1

#define WATCHDOG_CHANNEL_PERIODIC 0
#define WATCHDOG_CHANNEL_PERIODIC_TIMEOUT SYS_TICS_OF_USEC(30000)

/*  ********************************************************************/

#define BAT_VOLT_SCALE		15.58

/*  ********************************************************************/


//############### INCLUDE BOARDS #####################

#ifdef IMU_PIXHAWK_V200
#include "imu_conf_v200.h"
#endif

#ifdef IMU_PIXHAWK_V210
#include "imu_conf_v210.h"
#endif

#endif /* _CONF_H_ */
