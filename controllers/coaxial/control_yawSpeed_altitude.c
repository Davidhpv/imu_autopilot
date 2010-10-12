//10-Jun-2009 10:36:16
//29-May-2009 15:26:24
//29-May-2009 12:31:07
//28-May-2009 14:46:37
//28-May-2009 14:40:49
//12-May-2009 12:41:57
//12-May-2009 12:38:37
//12-May-2009 12:38:36
//12-May-2009 12:23:09
//12-May-2009 12:08:41
//12-May-2009 12:08:23
//12-May-2009 12:07:58
//12-May-2009 12:06:12
/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Joseph Kruijen, jkruijen@student.ethz.ch
Contributing Authors (in alphabetical order):



(c) 2008, 2009 PIXHAWK PROJECT

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

#include "control_yaw_altitude.h"

void control_yawSpeed_altitude( float yawSpeedRef, float yawSpeed, float z, float zRef, float* iU, float* iL) {

	// (!) Static variables are only initialized once, not at every call to this function
	// Static variables keep the same value from the last function call
	static float xk = 0;

	// Instantiate result variables with zero content
	float iMean = 0;
	float deltai = 0;
	float xNext = 0;
	float saturation=0;


	//Update controler with "update_control_yawSpeed_altitude.m"
	//L1
	zRef = zRef * 1;      //normalize
	z = z * 1;      //normalize
	iMean = 2.7482* xk +-18.8365* (zRef-z);
	iMean = iMean * 0.24234;      //denormalize
	yawSpeedRef = yawSpeedRef * 0.31831;      //normalize
	yawSpeed = yawSpeed * 0.31831;      //normalize
	deltai = 0.05*(yawSpeedRef-yawSpeed);
	deltai = deltai * 0.24234;      //denormalize
	//end_L1

	//L2
	xNext = 0.4017 * xk + 4* (zRef-z);
	//end_L2



	// Set the float numbers at the provided pointer
	// addresses to the calculated values
	// unnormation of normed rotor speed
	//transform wMean and deltaW to upper(wU)- and lower-rotorspeed(wL)
	//*iL = iMean+deltai;
	//*iU = iMean-deltai;


	//for simulink
	//*iU = iMean;
	//*iL = deltai;


	//uncoment for real use
	//anti reset windup/saturation
	if(iMean - deltai> 0.7f){
	*iL = 0.7f;
	saturation=1;}
	else *iL = iMean - deltai;
	if(iMean - deltai< 0.0f){
	*iL = 0.0f;
	saturation=1;}
	else *iL = iMean - deltai;

	if(iMean + deltai> 0.7f){
	*iU = 0.7f;
	saturation=1;}
	else *iU = iMean + deltai;
	if(iMean + deltai< 0.0f){
	*iU = 0.0f;
	saturation=1;}
	else *iU = iMean + deltai;

	//controller update
	if(saturation==1);// Do nothing xk=xk;
	else xk=xNext;

}
