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
 * @file
 *   @brief Implementation of Complimentary Attitude Filter
 *
 *   @author Joseph Kruijen <jkruijen@student.ethz.ch>
 *   @author Florian Zurbriggen <florianz@student.ethz.ch>
 *
 */

#include "attitude_compl_euler.h"
#include "inttypes.h"
#include "math.h"
#include "conf.h"

#define PI  3.1415926535897932384626433832795029

//values for motors unmounted
////////////////////////////////////////////////
//tuned for motors unmounted !!!!!!!!!!!!!!!!!!!
////////////////////////////////////////////////
//#define ACCEL_NEUTRALX = 32835
//#define ACCEL_NEUTRALY = 32760
//#define ACCEL_NEUTRALZ = 33144
//#define GYRO_NEUTRALX = 32182
//#define GYRO_NEUTRALY = 34345
//#define GYRO_NEUTRALZ = 32334

//values for motors mounted AND running
////////////////////////////////////////////////
//tuned for running motors!!!!!!!!!!!!!!!!!!!
////////////////////////////////////////////////

//values for PH 1 Board*****************************************************************************/

//#define ACCEL_NEUTRALX 5
//#define ACCEL_NEUTRALY -26
//#define ACCEL_NEUTRALZ 37
//#define GYRO_NEUTRALX 28916
//#define GYRO_NEUTRALY 29108
//#define GYRO_NEUTRALZ 29095
//
//#define MAG_NEUTRALX -56
//#define MAG_NEUTRALY -17
//#define MAG_SCALEX 0.0208333333f
//#define MAG_SCALEY 0.0196078431f
//#define DIP_ANGLE 1.0472f
//
//#define GYRO_SCALE_X 0.0011716218 //unit[(rad/sec)/Gyro_Step] FROM MEASURED DATA
//#define GYRO_SCALE_Y 0.0011303133
//#define GYRO_SCALE_Z 0.0008148486


//Lowpass paraameters****************************************************************************/
#define A_LOW 0.94
#define B_LOW 0.25
#define C_LOW 0.24
#define D_LOW 0
//(Integrate and Highpass) paraameters
#define A_HIGH 0.94
#define B_HIGH 0.125
#define C_HIGH 0.16
#define D_HIGH 0
//Rate moving average length
//#define LENGTH_RATE_LOWPASS 5

#if defined(IMU_PIXHAWK_V200) || defined(IMU_PIXHAWK_V210)

#define ACCEL_NEUTRALX -7
#define ACCEL_NEUTRALY -30
#define ACCEL_NEUTRALZ -1


//DONT USE! NOW PARAMS
#define GYRO_NEUTRALX 29780
#define GYRO_NEUTRALY 29849
#define GYRO_NEUTRALZ 29898
//#define GYRO_NEUTRALZ 29439


#define MAG_NEUTRALX -56
#define MAG_NEUTRALY -17
#define MAG_SCALEX 0.0208333333f
#define MAG_SCALEY 0.0196078431f
#define DIP_ANGLE 1.0472f

#define GYRO_SCALE_X 0.001007 //unit[(rad/sec)/Gyro_Step] FROM MEASURED DATA
#define GYRO_SCALE_Y 0.000990
#define GYRO_SCALE_Z 0.001002

void complimentary_filter_predict_rad(int16_vect3 accel_raw, uint16_vect3 gyro_raw, int16_vect3 mag_raw, float_vect3* attitude, float_vect3* att_rates){
	static float phi_acc_state = 0;
	static float theta_acc_state = 0;
	static float psi_mag_state = 0;
	static float phi_gyro_state = 0;
	static float theta_gyro_state = 0;
	static float psi_gyro_state = 0;
	static float phi_rate_state = 0;
	static float theta_rate_state = 0;
	static float psi_rate_state = 0;

	float phi;
	float theta;
	float psi;
	float phi_acc;
	float theta_acc;
	float psi_mag;
	float phi_gyro;
	float theta_gyro;
	float psi_gyro;
	static float psi_prev = 0;

//	static float phi_rate_array[LENGTH_RATE_LOWPASS];
//	static float theta_rate_array[LENGTH_RATE_LOWPASS];
//	static float psi_rate_array[LENGTH_RATE_LOWPASS];
//	static float phi_rate = 0;
//	static float theta_rate = 0;
//	static float psi_rate = 0;
//	static uint32_t rate_idx = 0;

	//unbias magnet sensor data
	float_vect3 mag;
    mag.x = -(mag_raw.x-MAG_NEUTRALX)*MAG_SCALEX;
    mag.y = -(mag_raw.y-MAG_NEUTRALY)*MAG_SCALEY;
    mag.z = mag_raw.z;

    //unbias acccel
	int32_vect3 accel;
	accel.x = -(accel_raw.x - ACCEL_NEUTRALX);	//flipping coordinate system from z upward to downward
	accel.y = -(accel_raw.y - ACCEL_NEUTRALY);
	accel.z = +(accel_raw.z - ACCEL_NEUTRALZ);

	//unbias gyro
	int32_vect3 gyro;
	gyro.x = -(gyro_raw.x - GYRO_NEUTRALX);
	gyro.y = +(gyro_raw.y - GYRO_NEUTRALY);
	gyro.z = -(gyro_raw.z - GYRO_NEUTRALZ);

	//scale gyro to rad/sec
	float_vect3 att_rates_raw;
	att_rates_raw.x = gyro.x * GYRO_SCALE_X;
	att_rates_raw.y = gyro.y * GYRO_SCALE_Y;
	att_rates_raw.z = gyro.z * GYRO_SCALE_Z;


	//lowpass Accelerations (maybe later)
	//lowpass magnet sensor data (maybe later)

	//Nick Roll measurement build
	float_vect3 attitude_tmp;
	attitude_tmp.x = fast_atan2(accel.y,accel.z);	//euler angle phi  fast_atan2
	attitude_tmp.y = -fast_atan2(accel.x,accel.z*(lookup_sin(attitude_tmp.x)+lookup_cos(attitude_tmp.x))); //euler angle theta



	//lowpass nick roll
	phi_acc =		C_LOW*phi_acc_state + D_LOW*attitude_tmp.x;
	phi_acc_state = A_LOW*phi_acc_state + B_LOW*attitude_tmp.x;

	theta_acc =			C_LOW*theta_acc_state + D_LOW*attitude_tmp.y;
	theta_acc_state =	A_LOW*theta_acc_state + B_LOW*attitude_tmp.y;

	//integrate and highpass nick roll from gyro
	phi_gyro =		 C_HIGH*phi_gyro_state + D_HIGH*att_rates_raw.x;
	phi_gyro_state = A_HIGH*phi_gyro_state + B_HIGH*att_rates_raw.x;

	theta_gyro =			C_HIGH*theta_gyro_state + D_HIGH*att_rates_raw.y;
	theta_gyro_state =	A_HIGH*theta_gyro_state + B_HIGH*att_rates_raw.y;

	//add up resulting high and lowpassed angles
	phi = phi_acc + phi_gyro;
	theta = theta_acc + theta_gyro;


	//test from tilt compensation algorithm for 2 axis magnetic compass
	mag.z=(sin(DIP_ANGLE)+mag.x*sin(theta)-mag.y*cos(theta)*sin(phi))/(cos(theta)*cos(phi));


	//Transformation of maget vector to inertial frame and computation of angle in the x-y plane
	attitude_tmp.z = fast_atan2(lookup_cos(theta)*mag.x + lookup_sin(phi)*lookup_sin(theta)*mag.y + lookup_cos(phi)*lookup_sin(theta)*mag.z , lookup_cos(phi)*mag.y - lookup_sin(phi)*mag.z);


//	//asembly of angle with range [-PI,PI] to a continous function with range [-2*PI,2*PI]
	if(psi_prev > PI/2 && attitude_tmp.z < psi_prev - PI)				attitude_tmp.z += 2*PI;
	else if(psi_prev < -PI/2 && attitude_tmp.z > psi_prev + PI)			attitude_tmp.z -= 2*PI;
	else if(attitude_tmp.z > PI/2 && psi_prev < attitude_tmp.z - PI)	attitude_tmp.z -= 2*PI;
	else if(attitude_tmp.z < -PI/2 && psi_prev > attitude_tmp.z +PI)	attitude_tmp.z += 2*PI;

	//Reset of angle to Zero if it reaches +-2*PI
	if (attitude_tmp.z > 2*PI){
 		attitude_tmp.z -= 2*PI;
 		psi_mag_state=0;
		}
	else if(attitude_tmp.z < -2*PI){
    	attitude_tmp.z += 2*PI;
    	psi_mag_state=0;
		}
	psi_prev = attitude_tmp.z;



	//lowpass psi_tmp
	psi_mag =		C_LOW*psi_mag_state + D_LOW*attitude_tmp.z;
	psi_mag_state = A_LOW*psi_mag_state + B_LOW*attitude_tmp.z;

	//integrate and highpass yaw from gyro
	psi_gyro =		 C_HIGH*psi_gyro_state + D_HIGH*att_rates_raw.z;
	psi_gyro_state = A_HIGH*psi_gyro_state + B_HIGH*att_rates_raw.z;

	//add up resulting high and lowpassed angles
	psi = psi_mag + psi_gyro;


//	//Signal trasformation from range [-2*PI,2*PI] to [-PI,PI]
    if(psi > PI)			psi = psi -2*PI;
    else if(psi < -PI)	psi = psi +2*PI;



	//output euler angles
	(*attitude).x = phi;
	(*attitude).y = theta;
	(*attitude).z = psi;



	//att_rates filtering for gyro rates output in SI units
	//lowpass version (0.1sec delay)
	(*att_rates).x=		C_LOW*phi_rate_state + D_LOW*att_rates_raw.x;
	phi_rate_state = A_LOW*phi_rate_state + B_LOW*att_rates_raw.x;


	(*att_rates).y=		C_LOW*theta_rate_state + D_LOW*att_rates_raw.y;
	theta_rate_state = A_LOW*theta_rate_state + B_LOW*att_rates_raw.y;

	(*att_rates).z=		C_LOW*psi_rate_state + D_LOW*att_rates_raw.z;
	psi_rate_state = A_LOW*psi_rate_state + B_LOW*att_rates_raw.z;
//	(*att_rates).z=		att_rates_raw.z;

	// threshold values < |0.008|
//	if (((*att_rates).z < 0.008) && (*att_rates).z > - 0.008){
//		(*att_rates).z = 0;
//	}



//	//att_rates filtering for gyro rates output in SI units
//	//moving average on gyros 0.05sec delay
//	phi_rate -= phi_rate_array[rate_idx]/LENGTH_RATE_LOWPASS;
//	theta_rate -= theta_rate_array[rate_idx]/LENGTH_RATE_LOWPASS;
//	psi_rate -= psi_rate_array[rate_idx]/LENGTH_RATE_LOWPASS;
//
//	phi_rate_array[rate_idx] = gyro.x;
//	theta_rate_array[rate_idx] = gyro.y;
//	psi_rate_array[rate_idx] = gyro.z;
//
//	phi_rate += phi_rate_array[rate_idx]/LENGTH_RATE_LOWPASS;
//	theta_rate += theta_rate_array[rate_idx]/LENGTH_RATE_LOWPASS;
//	psi_rate += psi_rate_array[rate_idx]/LENGTH_RATE_LOWPASS;
//	++rate_idx;
//	if(rate_idx>=LENGTH_RATE_LOWPASS) rate_idx=0;
//
//	(*att_rates).x = phi_rate;
//	(*att_rates).y = theta_rate;
//	(*att_rates).z = psi_rate;

}

#endif


#if defined(BOARD_PIXHAWK_V100) || defined(BOARD_TWOG_BOOZ)

void complimentary_filter_predict_rad(int16_vect3 accel_raw, uint16_vect3 gyro_raw, int16_vect3 mag_raw, float_vect3* attitude, float_vect3* att_rates){
	static float phi_acc_state = 0;
	static float theta_acc_state = 0;
	static float psi_mag_state = 0;
	static float phi_gyro_state = 0;
	static float theta_gyro_state = 0;
	static float psi_gyro_state = 0;
	static float phi_rate_state = 0;
	static float theta_rate_state = 0;
	static float psi_rate_state = 0;

	float phi;
	float theta;
	float psi;
	float phi_acc;
	float theta_acc;
	float psi_mag;
	float phi_gyro;
	float theta_gyro;
	float psi_gyro;
	static float psi_prev = 0;

//	static float phi_rate_array[LENGTH_RATE_LOWPASS];
//	static float theta_rate_array[LENGTH_RATE_LOWPASS];
//	static float psi_rate_array[LENGTH_RATE_LOWPASS];
//	static float phi_rate = 0;
//	static float theta_rate = 0;
//	static float psi_rate = 0;
//	static uint32_t rate_idx = 0;

	//unbias magnet sensor data
	float_vect3 mag;
    mag.x = (mag_raw.x-MAG_NEUTRALX)*MAG_SCALEX;
    mag.y = (mag_raw.y-MAG_NEUTRALY)*MAG_SCALEY;
    mag.z = mag_raw.z;

    //unbias acccel
	int32_vect3 accel;
	accel.x = +accel_raw.x - ACCEL_NEUTRALX;	//flipping coordinate system from z upward to downward
	accel.y = -accel_raw.y + ACCEL_NEUTRALY;
	accel.z = -accel_raw.z + ACCEL_NEUTRALZ;

	//unbias gyro
	int32_vect3 gyro;
	gyro.x = gyro_raw.x - GYRO_NEUTRALX;
	gyro.y = -gyro_raw.y + GYRO_NEUTRALY;
	gyro.z = gyro_raw.z - GYRO_NEUTRALZ;

	//scale gyro to rad/sec
	float_vect3 att_rates_raw;
	att_rates_raw.x = gyro.x * GYRO_SCALE_X;
	att_rates_raw.y = gyro.y * GYRO_SCALE_Y;
	att_rates_raw.z = gyro.z * GYRO_SCALE_Z;


	//lowpass Accelerations (maybe later)
	//lowpass magnet sensor data (maybe later)

	//Nick Roll measurement build
	float_vect3 attitude_tmp;
	attitude_tmp.x = fast_atan2(accel.y,accel.z);	//euler angle phi  fast_atan2
	attitude_tmp.y = -fast_atan2(accel.x,accel.z*(lookup_sin(attitude_tmp.x)+lookup_cos(attitude_tmp.x))); //euler angle theta



	//lowpass nick roll
	phi_acc =		C_LOW*phi_acc_state + D_LOW*attitude_tmp.x;
	phi_acc_state = A_LOW*phi_acc_state + B_LOW*attitude_tmp.x;

	theta_acc =			C_LOW*theta_acc_state + D_LOW*attitude_tmp.y;
	theta_acc_state =	A_LOW*theta_acc_state + B_LOW*attitude_tmp.y;

	//integrate and highpass nick roll from gyro
	phi_gyro =		 C_HIGH*phi_gyro_state + D_HIGH*att_rates_raw.x;
	phi_gyro_state = A_HIGH*phi_gyro_state + B_HIGH*att_rates_raw.x;

	theta_gyro =			C_HIGH*theta_gyro_state + D_HIGH*att_rates_raw.y;
	theta_gyro_state =	A_HIGH*theta_gyro_state + B_HIGH*att_rates_raw.y;

	//add up resulting high and lowpassed angles
	phi = phi_acc + phi_gyro;
	theta = theta_acc + theta_gyro;


	//test from tilt compensation algorithm for 2 axis magnetic compass
	mag.z=(sin(DIP_ANGLE)+mag.x*sin(theta)-mag.y*cos(theta)*sin(phi))/(cos(theta)*cos(phi));


	//Transformation of maget vector to inertial frame and computation of angle in the x-y plane
	attitude_tmp.z = fast_atan2(lookup_cos(theta)*mag.x + lookup_sin(phi)*lookup_sin(theta)*mag.y + lookup_cos(phi)*lookup_sin(theta)*mag.z , lookup_cos(phi)*mag.y - lookup_sin(phi)*mag.z);


//	//asembly of angle with range [-PI,PI] to a continous function with range [-2*PI,2*PI]
	if(psi_prev > PI/2 && attitude_tmp.z < psi_prev - PI)				attitude_tmp.z += 2*PI;
	else if(psi_prev < -PI/2 && attitude_tmp.z > psi_prev + PI)			attitude_tmp.z -= 2*PI;
	else if(attitude_tmp.z > PI/2 && psi_prev < attitude_tmp.z - PI)	attitude_tmp.z -= 2*PI;
	else if(attitude_tmp.z < -PI/2 && psi_prev > attitude_tmp.z +PI)	attitude_tmp.z += 2*PI;

	//Reset of angle to Zero if it reaches +-2*PI
	if (attitude_tmp.z > 2*PI){
 		attitude_tmp.z -= 2*PI;
 		psi_mag_state=0;
		}
	else if(attitude_tmp.z < -2*PI){
    	attitude_tmp.z += 2*PI;
    	psi_mag_state=0;
		}
	psi_prev = attitude_tmp.z;



	//lowpass psi_tmp
	psi_mag =		C_LOW*psi_mag_state + D_LOW*attitude_tmp.z;
	psi_mag_state = A_LOW*psi_mag_state + B_LOW*attitude_tmp.z;

	//integrate and highpass yaw from gyro
	psi_gyro =		 C_HIGH*psi_gyro_state + D_HIGH*att_rates_raw.z;
	psi_gyro_state = A_HIGH*psi_gyro_state + B_HIGH*att_rates_raw.z;

	//add up resulting high and lowpassed angles
	psi = psi_mag + psi_gyro;


//	//Signal trasformation from range [-2*PI,2*PI] to [-PI,PI]
    if(psi > PI)			psi = psi -2*PI;
    else if(psi < -PI)	psi = psi +2*PI;



	//output euler angles
	(*attitude).x = phi;
	(*attitude).y = theta;
	(*attitude).z = psi;



	//att_rates filtering for gyro rates output in SI units
	//lowpass version (0.1sec delay)
	(*att_rates).x=		C_LOW*phi_rate_state + D_LOW*att_rates_raw.x;
	phi_rate_state = A_LOW*phi_rate_state + B_LOW*att_rates_raw.x;


	(*att_rates).y=		C_LOW*theta_rate_state + D_LOW*att_rates_raw.y;
	theta_rate_state = A_LOW*theta_rate_state + B_LOW*att_rates_raw.y;

	(*att_rates).z=		C_LOW*psi_rate_state + D_LOW*att_rates_raw.z;
	psi_rate_state = A_LOW*psi_rate_state + B_LOW*att_rates_raw.z;
//	(*att_rates).z=		att_rates_raw.z;

	// threshold values < |0.008|
//	if (((*att_rates).z < 0.008) && (*att_rates).z > - 0.008){
//		(*att_rates).z = 0;
//	}



//	//att_rates filtering for gyro rates output in SI units
//	//moving average on gyros 0.05sec delay
//	phi_rate -= phi_rate_array[rate_idx]/LENGTH_RATE_LOWPASS;
//	theta_rate -= theta_rate_array[rate_idx]/LENGTH_RATE_LOWPASS;
//	psi_rate -= psi_rate_array[rate_idx]/LENGTH_RATE_LOWPASS;
//
//	phi_rate_array[rate_idx] = gyro.x;
//	theta_rate_array[rate_idx] = gyro.y;
//	psi_rate_array[rate_idx] = gyro.z;
//
//	phi_rate += phi_rate_array[rate_idx]/LENGTH_RATE_LOWPASS;
//	theta_rate += theta_rate_array[rate_idx]/LENGTH_RATE_LOWPASS;
//	psi_rate += psi_rate_array[rate_idx]/LENGTH_RATE_LOWPASS;
//	++rate_idx;
//	if(rate_idx>=LENGTH_RATE_LOWPASS) rate_idx=0;
//
//	(*att_rates).x = phi_rate;
//	(*att_rates).y = theta_rate;
//	(*att_rates).z = psi_rate;

}

#endif

