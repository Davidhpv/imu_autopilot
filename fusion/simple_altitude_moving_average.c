/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Joseph Kruijen <jkruijen@student.ethz.ch>
Contributing Authors (in alphabetical order):



(c) 2009 PIXHAWK PROJECT

This file is part of the PIXHAWK project

    mavlib is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    mavlib is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with mavlib. If not, see <http://www.gnu.org/licenses/>.

========================================================================*/

//#include "inttypes.h"
#include "stdlib.h"
#include "simple_altitude_moving_average.h"


#define LENGTH_HEIGHT_LOWPASS 50
#define LENGTH_HEIGHT_SPEED_LOWPASS 50

void simple_altitude_moving_average(float height_raw, float* height, float* heightSpeed){
	static float height_prev=0;
	static float height_low_prev=0;


	//from mm to m
	height_raw = -height_raw / 1000.0f;

	//outlyer removal
    if(height_raw -height_prev > 0.6f) height_raw = height_prev;
    if(height_raw -height_prev < -0.6f) height_raw = height_prev;


//  float heightSpeed_raw;
//	heightSpeed_raw = (height_raw - height_prev)/0.02-0.1;
//	//lowpass heightSpeed
//	static float heightSpeed_array[LENGTH_HEIGHT_SPEED_LOWPASS]; ///< The last n measurements, with n defined by LENGTH_ACCEL_LOWPASS
//	static float lowpass_heightSpeed;
//	static uint32_t indexSpeed = 0;
//	lowpass_heightSpeed -= heightSpeed_array[indexSpeed]/LENGTH_HEIGHT_SPEED_LOWPASS;
//	heightSpeed_array[indexSpeed] = heightSpeed_raw;
//	lowpass_heightSpeed += heightSpeed_array[indexSpeed]/LENGTH_HEIGHT_SPEED_LOWPASS;
//
//	++indexSpeed;
//	if(indexSpeed>LENGTH_HEIGHT_SPEED_LOWPASS) indexSpeed=0;
//
//	*heightSpeed = lowpass_heightSpeed;



	//lowpass height
	static float height_array[LENGTH_HEIGHT_LOWPASS]; ///< The last n measurements, with n defined by LENGTH_ACCEL_LOWPASS
	static float lowpass_height;
	static uint32_t index_lowpass = 0;
	lowpass_height -= height_array[index_lowpass]/LENGTH_HEIGHT_LOWPASS;
	height_array[index_lowpass] = height_raw;
	lowpass_height += height_array[index_lowpass]/LENGTH_HEIGHT_LOWPASS;

	++index_lowpass;
	if(index_lowpass>LENGTH_HEIGHT_LOWPASS) index_lowpass=0;

	*height = lowpass_height;

	*heightSpeed = (lowpass_height - height_low_prev)/0.02;

	height_prev = height_raw;
	height_low_prev = lowpass_height;
}
