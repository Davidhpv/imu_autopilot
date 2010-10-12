//02-Jul-2009 17:09:29
//30-Jun-2009 18:02:51
//29-Jun-2009 16:54:01
//29-Jun-2009 16:03:06
//29-Jun-2009 09:29:24
//17-Jun-2009 12:36:57
//10-Jun-2009 10:40:29
//09-Jun-2009 13:10:36
//09-Jun-2009 13:09:30
//09-Jun-2009 13:08:45
//12-May-2009 12:41:07
//12-May-2009 12:40:24
//12-May-2009 12:40:24
//12-May-2009 12:40:23
//12-May-2009 12:38:40
//12-May-2009 12:22:19
//12-May-2009 12:21:12
//12-May-2009 12:20:23
//12-May-2009 12:19:12
//12-May-2009 12:18:16
//11-May-2009 15:07:45
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

#include "control_attitude.h"
#include <math.h>
#include "radio_control.h"
#include "conf.h"

void control_attitude(float roll, float rollRef, float rollSpeed, float rollSpeedRef, float nick, float nickRef, float nickSpeed, float nickSpeedRef, float* setServoPosY, float* setServoNegY){

	// Instantiate result variables with zero content
	float a = 0;		//swashplate angle (nick)
	float b = 0;		//swashplate angle (roll)

	//rollRef=rollRef+ROLL_TRIMM;
	//nickRef=nickRef+NICK_TRIMM;

	//Update controler with "update_control_attitude.m"
	//L1
	nickRef = nickRef * 1.9099;      //normalize
	nick = nick * 1.9099;      //normalize
	nickSpeedRef = nickSpeedRef * 0.31831;      //normalize
	nickSpeed = nickSpeed * 0.31831;      //normalize
	rollRef = rollRef * 1.9099;      //normalize
	roll = roll * 1.9099;      //normalize
	rollSpeedRef = rollSpeedRef * 0.31831;      //normalize
	rollSpeed = rollSpeed * 0.31831;      //normalize

	//mehr control (besser)
//	a = 0.1700 * (nickRef-nick) + 0.4808 * (nickSpeedRef-nickSpeed) +0.0539 * (rollRef-roll) +0.076 * (rollSpeedRef-rollSpeed) ;
//	b = -0.0565 * (nickRef-nick) - 0.1596 * (nickSpeedRef-nickSpeed) + 0.1623 * (rollRef-roll) + 0.2288 * (rollSpeedRef-rollSpeed) ;

	//R=500
//	a = 0.3336 * (nickRef-nick) + 0.6846 * (nickSpeedRef-nickSpeed) + 0.102 * (rollRef-roll) + 0.1116 * (rollSpeedRef-rollSpeed) ;
//	b = -0.1108 * (nickRef-nick) - 0.2273 * (nickSpeedRef-nickSpeed) + 0.3100 * (rollRef-roll) + 0.3361 * (rollSpeedRef-rollSpeed) ;

	//R=250, perfect results with last battery
	a = 0.466 * (nickRef-nick) + 0.8145 * (nickSpeedRef-nickSpeed) + 0.1412 * (rollRef-roll) + 0.1337 * (rollSpeedRef-rollSpeed);
	b = -0.1547 * (nickRef-nick) - 0.2705 * (nickSpeedRef-nickSpeed) + 0.4254 * (rollRef-roll) + 0.4026 * (rollSpeedRef-rollSpeed);


	//R=250, new weight
//	a = 0.4664 * (nickRef-nick) + 0.8245 * (nickSpeedRef-nickSpeed) + 0.1414 * (rollRef-roll) + 0.1344 * (rollSpeedRef-rollSpeed) ;
//	b = -0.1549 * (nickRef-nick) - 0.2738 * (nickSpeedRef-nickSpeed) + 0.4258 * (rollRef-roll) + 0.4046 * (rollSpeedRef-rollSpeed) ;
	//end_L1

	//R=350, new weight
//	a = 0.3966 * (nickRef-nick) + 0.7577 * (nickSpeedRef-nickSpeed) + 0.1214 * (rollRef-roll) + 0.1231 * (rollSpeedRef-rollSpeed) ;
//	b = -0.1317 * (nickRef-nick) -0.2516 * (nickSpeedRef-nickSpeed) + 0.3655 * (rollRef-roll) + 0.3706 * (rollSpeedRef-rollSpeed) ;
	//end_L1

	//R=350, new weight
//	a = 0.3338 * (nickRef-nick) + 0.6930 * (nickSpeedRef-nickSpeed) + 0.1030 * (rollRef-roll) + 0.1122 * (rollSpeedRef-rollSpeed) ;
//	b = -0.1109 * (nickRef-nick) -0.2301 * (nickSpeedRef-nickSpeed) + 0.3102 * (rollRef-roll) + 0.3379 * (rollSpeedRef-rollSpeed) ;
	//end_L1


	//saturation of normalized swashplate angles
	if(a>1) a=1;
	else if(a<-1) a=-1;
	if(b>1) b=1;
	else if(b<-1) b=-1;

	//L2
	a = (a) * 0.34907;      //denormalize
	b = (-b) * 0.34907;      //denormalize
	//end_L2

	//for simulink
	//*setServoLeft = a;
	//*setServoRight = b;


	//uncoment for real use
	//conversion from swashplate angles to servo angles for current mechanical setup (original Walkera servo position)

		//Bastel-Hubschrauber
//		if ((a + b)*2.15 - 0.05>=1) *setServoNegY = -0.872664626;  //50 degree
//		else if((a + b)*2.15 - 0.05<=-1) *setServoNegY = 0.872664626; //50 degree
//		else *setServoNegY = 1.0*acos((a + b)*2.15 - 0.05) - 0.07 + (a + b)*(a + b)/2.5 - 3.1416/2;		//Servo at position +y koordinate value in [rad]
//
//		if((-a + b)*2.15 + 0.03>=1) *setServoPosY = 0.872664626; //50 degree
//		else if((-a + b)*2.15 + 0.03<=-1) *setServoPosY = -0.872664626; //50 degree
//		else *setServoPosY = 1.0*asin((-a + b)*2.15 + 0.03) - 0.05 + (-a + b)*(-a + b)/2;				//Servo at position -y koordinate     value in [rad]


		//First prototype
		if ((a + b)*2.12 - 0.18>=1) *setServoNegY = -0.872664626;  //50 degree
		else if((a + b)*2.12 - 0.18<=-1) *setServoNegY = 0.872664626; //50 degree
		else *setServoNegY = 1.0*acos((a + b)*2.12 - 0.18) - 0.2 + (a + b)*(a + b)/10 - 3.1415/2;		//Servo at position +y koordinate value in [rad]

		if((-a + b)*2.15 + 0.15>=1) *setServoPosY = 0.872664626; //50 degree
		else if((-a + b)*2.15 + 0.15<=-1) *setServoPosY = -0.872664626; //50 degree
		else *setServoPosY = 1.0*asin((-a + b)*2.15 + 0.1) - 0.1 + (-a + b)*(-a + b)/2.25;				//Servo at position -y koordinate     value in [rad]

}

