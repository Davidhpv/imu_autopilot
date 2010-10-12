//10-Jun-2009 10:38:37
//10-Jun-2009 10:27:53
//10-Jun-2009 10:25:44
//08-Jun-2009 17:35:49
//08-Jun-2009 16:43:17
//08-Jun-2009 16:40:24
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
#define PI  3.1415926535897932384626433832795029
#define HOVER_INPUT 0.606

void control_yaw_altitude( float yawRef, float yaw, float yawDot, float zRef, float z,float zDot, float* iU, float* iL) {

	// (!) Static variables are only initialized once, not at every call to this function
	// Static variables keep the same value from the last function call
	float zDotRef = 0;
	float yawDotRef = 0;

	// Instantiate result variables with zero content
	float iMean = 0;
	float deltai = 0;

	//make sure angles are between +-pi
	if(yaw > PI) yaw = PI;
	if(yaw < -PI) yaw = -PI;
	if(yawRef > PI) yawRef = PI;
	if(yawRef > PI) yawRef = -PI;

	//manage -PI to PI value jumps
	//make sure control difference is smaller than PI (yawRef - yaw<PI)
	if(yawRef - yaw > PI) yaw +=2*PI;
	if(yawRef - yaw < -PI) yaw -=2*PI;
	//old version
//	if(yaw > PI/2 && yawRef < yaw - PI)				yaw = yaw - 2*PI;
//	else if(yaw < -PI/2 && yawRef > yaw + PI)		yaw = yaw + 2*PI;
//	else if(yawRef > PI/2 && yaw < yawRef - PI)		yaw = yaw + 2*PI;
//  else if(yawRef < -PI/2 && yaw > yawRef +PI)		yaw = yaw - 2*PI;


	//Update controler with "update_control_yaw_altitude.m"
	//L1
	zRef = zRef * 1;		//normalize
	z = z * 1;				//normalize
	zDot = zDot * 1;		//normalize

	zDotRef = 1*0.5/4*1* (zRef-z);
	iMean = 1*-0.5*1/0.3029* (zDotRef-zDot);
	iMean = iMean * 0.3029;      //denormalize


	yawRef = yawRef * 1.9099;      //normalize
	yaw = yaw * 1.9099;			//normalize
	yawDot = yawDot * 0.31831;			//normalize

	yawDotRef = 1/1.9099*1*0.31831 * (yawRef-yaw);
	deltai =  1/0.31831*4*1/0.3029 * (yawDotRef-yawDot);
	deltai = deltai *  0.3029;      //denormalize
	//end_L1

	//L2
	//nothing here
	//end_L2

    // MATLAB uses LCC per default
	// So this should be selected automatically correct
	#ifdef __LCC__
	//for simulink
	//*iU = iMean;
	//*iL = deltai;
	#else



	//yaw limitation for bether system separation of alt/yaw
	if(deltai > 0.7-HOVER_INPUT) deltai = 0.7-HOVER_INPUT;
	if(deltai < -0.7+HOVER_INPUT) deltai = -0.7+HOVER_INPUT;

	//uncoment for real use
	//anti reset windup/saturation & add HOVER_INPUT to output
	//lower rotor
	if(iMean - deltai> 0.7f-HOVER_INPUT){*iL = 0.2f +HOVER_INPUT;}
	else 								*iL = iMean - deltai +HOVER_INPUT;
	if(iMean - deltai< -0.4f)			{*iL = -0.2f +HOVER_INPUT;}
	else 								*iL = iMean - deltai +HOVER_INPUT;
	//upper rotor
	if(iMean + deltai> 0.7f-HOVER_INPUT){*iU = 0.2f +HOVER_INPUT;}
	else 								*iU = iMean + deltai + HOVER_INPUT;
	if(iMean + deltai< -0.4f)			{*iU = -0.2f +HOVER_INPUT;}
	else 								*iU = iMean + deltai +HOVER_INPUT;
	#endif

}
