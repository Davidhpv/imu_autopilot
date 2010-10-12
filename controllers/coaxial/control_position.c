//17-Jun-2009 13:03:12
//16-Jun-2009 16:56:16
//12-May-2009 12:43:30
//12-May-2009 12:38:54
//12-May-2009 12:38:48
//12-May-2009 12:33:24
//12-May-2009 12:32:17
//12-May-2009 12:32:03
//12-May-2009 12:31:14
//12-May-2009 12:31:12
//11-May-2009 16:31:44
//11-May-2009 16:31:00
//11-May-2009 16:30:07
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
#include "control_position.h"
#include "radio_control.h"
#include "global_data.h"


void control_position(float x, float xRef, float xSpeed, float xSpeedRef, float y, float yRef, float ySpeed, float ySpeedRef, float nick, float nickSpeed, float roll, float rollSpeed, float* nickRef, float* nickSpeedRef, float* rollRef, float* rollSpeedRef){
//const float pi=3.1416;
	float nickRefTmp=0;
	float nickSpeedRefTmp=0;
	float rollRefTmp=0;
	float rollSpeedRefTmp=0;

	//Update controler with "update_control_position.m" 
	//L1
	x = x * 1.0;      //normalize
	xRef = xRef * 1.0;      //normalize
	xSpeed = xSpeed * 1.0;      //normalize
	xSpeedRef = xSpeedRef * 1.0;      //normalize
	y = y * 1.0;      //normalize
	yRef = yRef * 1.0;      //normalize
	ySpeed = ySpeed * 1.0;      //normalize
	ySpeedRef = ySpeedRef * 1.0;      //normalize
	nick = nick * 1.9099;      //normalize 
	nickSpeed = nickSpeed * 0.31831;      //normalize 
	roll = roll * 1.9099;      //normalize 
	rollSpeed = rollSpeed * 0.31831;      //normalize 

	//R=10 fast
	nickRefTmp =      -0.1523* (xRef-x) +  -0.1615* (xSpeedRef-xSpeed) + -0.0077* (yRef-y) + -0.0077* (ySpeedRef-ySpeed) + 0.2115* (-nick) + 0.1614*  (-nickSpeed) + -0.0053* (-roll) + 0.0002* (-rollSpeed);
	nickSpeedRefTmp = -0.2663* (xRef-x) +  -0.2823* (xSpeedRef-xSpeed) + -0.0134* (yRef-y) + -0.0134* (ySpeedRef-ySpeed) + 0.3697* (-nick) + 0.2821*  (-nickSpeed) + -0.0092* (-roll) + 0.0003* (-rollSpeed);
	rollRefTmp =      -0.0113* (xRef-x) +  -0.0130* (xSpeedRef-xSpeed) + 0.2239*  (yRef-y) + 0.2450*  (ySpeedRef-ySpeed) + 0.0144* (-nick) + -0.0012* (-nickSpeed) + 0.1866*  (-roll) + 0.0689* (-rollSpeed);
	rollSpeedRefTmp = -0.0107* (xRef-x) +  -0.0123* (xSpeedRef-xSpeed) + 0.2119*  (yRef-y) + 0.2319*  (ySpeedRef-ySpeed) + 0.0137* (-nick) + -0.0012* (-nickSpeed) + 0.1766 * (-roll) + 0.0652* (-rollSpeed);


	//R=100 slower
//	nickRefTmp =      -0.0488* (xRef-x) +  -0.0783* (xSpeedRef-xSpeed) + -0.0022* (yRef-y) + -0.0034* (ySpeedRef-ySpeed) + 0.1075* (-nick) + 0.0878*  (-nickSpeed) + -0.0025* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp = -0.0854* (xRef-x) +  -0.1368* (xSpeedRef-xSpeed) + -0.0039* (yRef-y) + -0.0059* (ySpeedRef-ySpeed) + 0.1879* (-nick) + 0.1535*  (-nickSpeed) + -0.0043* (-roll) + 0.0002* (-rollSpeed);
//	rollRefTmp =      -0.0033* (xRef-x) +  -0.0060* (xSpeedRef-xSpeed) + 0.1262*  (yRef-y) + 0.1262*  (ySpeedRef-ySpeed) + 0.0076* (-nick) + -0.0006* (-nickSpeed) + 0.0978*  (-roll) + 0.0369* (-rollSpeed);
//	rollSpeedRefTmp = -0.0031* (xRef-x) +  -0.0057* (xSpeedRef-xSpeed) + 0.0678*  (yRef-y) + 0.1194*  (ySpeedRef-ySpeed) + 0.0072* (-nick) + -0.0005* (-nickSpeed) + 0.0925 * (-roll) + 0.0349* (-rollSpeed);

	//R=250 more slower
//	nickRefTmp =      -0.0310* (xRef-x) +  -0.0598* (xSpeedRef-xSpeed) + -0.0014* (yRef-y) + -0.0025* (ySpeedRef-ySpeed) + 0.0834* (-nick) + 0.0694*  (-nickSpeed) + -0.0019* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp = -0.0542* (xRef-x) +  -0.1045* (xSpeedRef-xSpeed) + -0.0024* (yRef-y) + -0.0044* (ySpeedRef-ySpeed) + 0.1458* (-nick) + 0.1212*  (-nickSpeed) + -0.0033* (-roll) + 0.0002* (-rollSpeed);
//	rollRefTmp =      -0.0020* (xRef-x) +  -0.0045* (xSpeedRef-xSpeed) + 0.0454*  (yRef-y) + 0.0982*  (ySpeedRef-ySpeed) + 0.0059* (-nick) + -0.0004* (-nickSpeed) + 0.0767*  (-roll) + 0.0291* (-rollSpeed);
//	rollSpeedRefTmp = -0.0019* (xRef-x) +  -0.0042* (xSpeedRef-xSpeed) + 0.0430*  (yRef-y) + 0.0930*  (ySpeedRef-ySpeed) + 0.0056* (-nick) + -0.0004* (-nickSpeed) + 0.0726 * (-roll) + 0.0276* (-rollSpeed);

	//R=250 new weight
//	nickRefTmp =      -0.0307* (xRef-x) +  -0.0592* (xSpeedRef-xSpeed) + -0.0014* (yRef-y) + -0.0025* (ySpeedRef-ySpeed) + 0.0834* (-nick) + 0.0702*  (-nickSpeed) + -0.0019* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp = -0.0543* (xRef-x) +  -0.1046* (xSpeedRef-xSpeed) + -0.0024* (yRef-y) + -0.0044* (ySpeedRef-ySpeed) + 0.1474* (-nick) + 0.1240*  (-nickSpeed) + -0.0033* (-roll) + 0.0002* (-rollSpeed);
//	rollRefTmp =      -0.0020* (xRef-x) +  -0.0045* (xSpeedRef-xSpeed) + 0.0453*  (yRef-y) + 0.0980*  (ySpeedRef-ySpeed) + 0.0060* (-nick) + -0.0004* (-nickSpeed) + 0.0768*  (-roll) + 0.0294* (-rollSpeed);
//	rollSpeedRefTmp = -0.0019* (xRef-x) +  -0.0043* (xSpeedRef-xSpeed) + 0.0431*  (yRef-y) + 0.0931*  (ySpeedRef-ySpeed) + 0.0057* (-nick) + -0.0004* (-nickSpeed) + 0.0730 * (-roll) + 0.0279* (-rollSpeed);

//	float channel_8 = radio_control_get_channel(8);
//	float channel_6 = radio_control_get_channel(6);
//
//	if((channel_8<0.0f) && (channel_6<-0.5f)){
//	//R=300
//	nickRefTmp =      -0.0283* (xRef-x) +  -0.0568* (xSpeedRef-xSpeed) + -0.0012* (yRef-y) + -0.0024* (ySpeedRef-ySpeed) + 0.0794* (-nick) + 0.0662*  (-nickSpeed) + -0.0018* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp = -0.0495* (xRef-x) +  -0.0992* (xSpeedRef-xSpeed) + -0.0022* (yRef-y) + -0.0042* (ySpeedRef-ySpeed) + 0.1387* (-nick) + 0.1157*  (-nickSpeed) + -0.0031* (-roll) + 0.0002* (-rollSpeed);
//	rollRefTmp =      -0.0018* (xRef-x) +  -0.0043* (xSpeedRef-xSpeed) + 0.0415*  (yRef-y) + 0.0935*  (ySpeedRef-ySpeed) + 0.0056* (-nick) + -0.0004* (-nickSpeed) + 0.0731*  (-roll) + 0.0278* (-rollSpeed);
//	rollSpeedRefTmp = -0.0017* (xRef-x) +  -0.0041* (xSpeedRef-xSpeed) + 0.0393*  (yRef-y) + 0.0885*  (ySpeedRef-ySpeed) + 0.0053* (-nick) + -0.0004* (-nickSpeed) + 0.0692 * (-roll) + 0.0263* (-rollSpeed);
//	global_data.position_controller_strenght=300;
//	}
//	if((channel_8<0.0f) && (channel_6<0.5f) && (channel_6>-0.5f)){
//	//R=400
//	nickRefTmp =      -0.0245* (xRef-x) +  -0.0523* (xSpeedRef-xSpeed) + -0.0011* (yRef-y) + -0.0022* (ySpeedRef-ySpeed) + 0.0734* (-nick) + 0.0615*  (-nickSpeed) + -0.0016* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp = -0.0429* (xRef-x) +  -0.0914* (xSpeedRef-xSpeed) + -0.0019* (yRef-y) + -0.0038* (ySpeedRef-ySpeed) + 0.1283* (-nick) + 0.1075*  (-nickSpeed) + -0.0029* (-roll) + 0.0001* (-rollSpeed);
//	rollRefTmp =      -0.0016* (xRef-x) +  -0.0039* (xSpeedRef-xSpeed) + 0.0360*  (yRef-y) + 0.0866*  (ySpeedRef-ySpeed) + 0.0052* (-nick) + -0.0004* (-nickSpeed) + 0.0678*  (-roll) + 0.0258* (-rollSpeed);
//	rollSpeedRefTmp = -0.0015* (xRef-x) +  -0.0037* (xSpeedRef-xSpeed) + 0.0340*  (yRef-y) + 0.0819*  (ySpeedRef-ySpeed) + 0.0049* (-nick) + -0.0004* (-nickSpeed) + 0.0642 * (-roll) + 0.0245* (-rollSpeed);
//	global_data.position_controller_strenght=400;
//	}
//	if((channel_8<0.0f) && (channel_6>0.5f)){
//		//R=500
//	nickRefTmp =      -0.0220* (xRef-x) +  -0.0491* (xSpeedRef-xSpeed) + -0.0010* (yRef-y) + -0.0020* (ySpeedRef-ySpeed) + 0.0691* (-nick) + 0.0581*  (-nickSpeed) + -0.0015* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp = -0.0384* (xRef-x) +  -0.0858* (xSpeedRef-xSpeed) + -0.0017* (yRef-y) + -0.0036* (ySpeedRef-ySpeed) + 0.1208* (-nick) + 0.1016*  (-nickSpeed) + -0.0027* (-roll) + 0.0001* (-rollSpeed);
//	rollRefTmp =      -0.0014* (xRef-x) +  -0.0037* (xSpeedRef-xSpeed) + 0.0322*  (yRef-y) + 0.0816*  (ySpeedRef-ySpeed) + 0.0049* (-nick) + -0.0004* (-nickSpeed) + 0.0640*  (-roll) + 0.0244* (-rollSpeed);
//	rollSpeedRefTmp = -0.0013* (xRef-x) +  -0.0035* (xSpeedRef-xSpeed) + 0.0304*  (yRef-y) + 0.0772*  (ySpeedRef-ySpeed) + 0.0047* (-nick) + -0.0003* (-nickSpeed) + 0.0606 * (-roll) + 0.0231* (-rollSpeed);
//	global_data.position_controller_strenght=500;
//	}
//	if((channel_8>0.0f) && (channel_6<-0.5f)){
//	//R=600
//	nickRefTmp =      -0.0201* (xRef-x) +  -0.0466* (xSpeedRef-xSpeed) + -0.0009* (yRef-y) + -0.0019* (ySpeedRef-ySpeed) + 0.0658* (-nick) + 0.0555*  (-nickSpeed) + -0.0015* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp = -0.0351* (xRef-x) +  -0.0815* (xSpeedRef-xSpeed) + -0.0015* (yRef-y) + -0.0034* (ySpeedRef-ySpeed) + 0.1150* (-nick) + 0.0970*  (-nickSpeed) + -0.0026* (-roll) + 0.0001* (-rollSpeed);
//	rollRefTmp =      -0.0013* (xRef-x) +  -0.0035* (xSpeedRef-xSpeed) + 0.0294*  (yRef-y) + 0.0777*  (ySpeedRef-ySpeed) + 0.0047* (-nick) + -0.0003* (-nickSpeed) + 0.0610*  (-roll) + 0.0233* (-rollSpeed);
//	rollSpeedRefTmp = -0.0012* (xRef-x) +  -0.0033* (xSpeedRef-xSpeed) + 0.0278*  (yRef-y) + 0.0735*  (ySpeedRef-ySpeed) + 0.0044* (-nick) + -0.0003* (-nickSpeed) + 0.0578 * (-roll) + 0.0221* (-rollSpeed);
//	global_data.position_controller_strenght=600;
//	}
//	if((channel_8>0.0f) && (channel_6<0.5f) && (channel_6>-0.5f)){
//	//R=700
//	nickRefTmp =      -0.0186* (xRef-x) +  -0.0446* (xSpeedRef-xSpeed) + -0.0008* (yRef-y) + -0.0019* (ySpeedRef-ySpeed) + 0.0631* (-nick) + 0.0533*  (-nickSpeed) + -0.0014* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp = -0.0325* (xRef-x) +  -0.0780* (xSpeedRef-xSpeed) + -0.0014* (yRef-y) + -0.0032* (ySpeedRef-ySpeed) + 0.1104* (-nick) + 0.0933*  (-nickSpeed) + -0.0025* (-roll) + 0.0001* (-rollSpeed);
//	rollRefTmp =      -0.0012* (xRef-x) +  -0.0033* (xSpeedRef-xSpeed) + 0.0272*  (yRef-y) + 0.0746*  (ySpeedRef-ySpeed) + 0.0045* (-nick) + -0.0003* (-nickSpeed) + 0.0587*  (-roll) + 0.0244* (-rollSpeed);
//	rollSpeedRefTmp = -0.0011* (xRef-x) +  -0.0032* (xSpeedRef-xSpeed) + 0.0258*  (yRef-y) + 0.0706*  (ySpeedRef-ySpeed) + 0.0043* (-nick) + -0.0003* (-nickSpeed) + 0.0555 * (-roll) + 0.0212* (-rollSpeed);
//	global_data.position_controller_strenght=700;
//	}
//	if((channel_8>0.0f) && (channel_6>0.5)){
//	//R=800
//	nickRefTmp = -0.0174 * (xRef - x) + -0.0430 * (xSpeedRef - xSpeed) + -0.0008 * (yRef - y) + -0.0018 * (ySpeedRef - ySpeed) + 0.0609 * (-nick) + 0.0516 * (-nickSpeed) + -0.0014 * (-roll) + 0.0001 * (-rollSpeed);
//	nickSpeedRefTmp = -0.0304 * (xRef - x) + -0.0752 * (xSpeedRef - xSpeed) + -0.0013 * (yRef - y) + -0.0031 * (ySpeedRef - ySpeed) + 0.1065 * (-nick) + 0.0901 * (-nickSpeed) + -0.0024 * (-roll) + 0.0001 * (-rollSpeed);
//	rollRefTmp = -0.0011 * (xRef - x) + -0.0032 * (xSpeedRef - xSpeed) + 0.0255 * (yRef - y) + 0.0720 * (ySpeedRef - ySpeed) + 0.0043 * (-nick) + -0.0003 * (-nickSpeed) + 0.0567 * (-roll) + 0.0217 * (-rollSpeed);
//	rollSpeedRefTmp = -0.0010* (xRef-x) +  -0.0030* (xSpeedRef-xSpeed) +   0.0241* (yRef-y)  +  0.0681 * (ySpeedRef-ySpeed) +  0.0041 * (-nick) + -0.0003*  (-nickSpeed)  +  0.0536 * (-roll)  + 0.0205* (-rollSpeed);
//	global_data.position_controller_strenght=800;
//	}

	//R=900
//	nickRefTmp =      -0.0164* (xRef-x) +  -0.0416* (xSpeedRef-xSpeed) + -0.0007* (yRef-y) + -0.0017* (ySpeedRef-ySpeed) + 0.0590* (-nick) + 0.0501*  (-nickSpeed) + -0.0013* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp = -0.0286* (xRef-x) +  -0.0728* (xSpeedRef-xSpeed) + -0.0012* (yRef-y) + -0.0030* (ySpeedRef-ySpeed) + 0.1032* (-nick) + 0.0875*  (-nickSpeed) + -0.0023* (-roll) + 0.0001* (-rollSpeed);
//	rollRefTmp =      -0.0010* (xRef-x) +  -0.0031* (xSpeedRef-xSpeed) + 0.0240*  (yRef-y) + 0.0698*  (ySpeedRef-ySpeed) + 0.0042* (-nick) + -0.0003* (-nickSpeed) + 0.0550*  (-roll) + 0.0210* (-rollSpeed);
//	rollSpeedRefTmp = -0.0010* (xRef-x) +  -0.0029* (xSpeedRef-xSpeed) + 0.0227*  (yRef-y) + 0.0661*  (ySpeedRef-ySpeed) + 0.0040* (-nick) + -0.0003* (-nickSpeed) + 0.0520 * (-roll) + 0.0199* (-rollSpeed);

	//R=1000
//	nickRefTmp =       -0.0156* (xRef-x) +   -0.0404* (xSpeedRef-xSpeed) + -0.0007* (yRef-y) + -0.0017* (ySpeedRef-ySpeed) +  0.0574* (-nick) + 0.0487*  (-nickSpeed) + -0.0013* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp =  -0.0272* (xRef-x) +    -0.0707* (xSpeedRef-xSpeed) +  -0.0012* (yRef-y) + -0.0029* (ySpeedRef-ySpeed) +  0.1004* (-nick) + 0.0852*  (-nickSpeed) + -0.0022* (-roll) + 0.0001* (-rollSpeed);
//	rollRefTmp =      -0.0010* (xRef-x) +  -0.0030* (xSpeedRef-xSpeed) +  0.0228*  (yRef-y) +  0.0679*  (ySpeedRef-ySpeed) +  0.0041* (-nick) + -0.0003* (-nickSpeed) +  0.0535*  (-roll) +  0.0205* (-rollSpeed);
//	rollSpeedRefTmp = -0.0009* (xRef-x) +  -0.0029* (xSpeedRef-xSpeed) + 0.0216*  (yRef-y) + 0.0642*  (ySpeedRef-ySpeed) + 0.0039* (-nick) + -0.0003* (-nickSpeed) + 0.0506 * (-roll) + 0.0194* (-rollSpeed);

	//R=1100
//	nickRefTmp =       -0.0148* (xRef-x) +   -0.0394* (xSpeedRef-xSpeed) + -0.0006* (yRef-y) + -0.0016* (ySpeedRef-ySpeed) +  0.0560* (-nick) + 0.0476*  (-nickSpeed) + -0.0012* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp =  -0.0259* (xRef-x) +    -0.0688* (xSpeedRef-xSpeed) +  -0.0011* (yRef-y) + -0.0028* (ySpeedRef-ySpeed) +  0.0979* (-nick) + 0.0832  (-nickSpeed) + -0.0022* (-roll) + 0.0001* (-rollSpeed);
//	rollRefTmp =      -0.0009* (xRef-x) +  -0.0029* (xSpeedRef-xSpeed) +  0.0217*  (yRef-y) +  0.0662*  (ySpeedRef-ySpeed) +  0.0040* (-nick) + -0.0003* (-nickSpeed) +  0.0522*  (-roll) +  0.0200* (-rollSpeed);
//	rollSpeedRefTmp = -0.0009* (xRef-x) +  -0.0028* (xSpeedRef-xSpeed) + 0.0206*  (yRef-y) + 0.0626*  (ySpeedRef-ySpeed) + 0.0038* (-nick) + -0.0003* (-nickSpeed) + 0.0494 * (-roll) + 0.0189* (-rollSpeed);

	//R=1200
//	nickRefTmp =       -0.0142* (xRef-x) +   -0.0384* (xSpeedRef-xSpeed) + -0.0006* (yRef-y) + -0.0016* (ySpeedRef-ySpeed) +  0.0547* (-nick) + 0.0465*  (-nickSpeed) + -0.0012* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp =  -0.0248* (xRef-x) +    -0.0672* (xSpeedRef-xSpeed) +  -0.0011* (yRef-y) + -0.0028* (ySpeedRef-ySpeed) +  0.0956* (-nick) + 0.0814*  (-nickSpeed) + -0.0021* (-roll) + 0.0001* (-rollSpeed);
//	rollRefTmp =      -0.0009* (xRef-x) +  -0.0029* (xSpeedRef-xSpeed) +  0.0208*  (yRef-y) +  0.0647*  (ySpeedRef-ySpeed) +  0.0039* (-nick) + -0.0003* (-nickSpeed) +  0.0511*  (-roll) +  0.0196* (-rollSpeed);
//	rollSpeedRefTmp = -0.0008* (xRef-x) +  -0.0027* (xSpeedRef-xSpeed) + 0.0197*  (yRef-y) + 0.0612*  (ySpeedRef-ySpeed) + 0.0037* (-nick) + -0.0003* (-nickSpeed) + 0.0483 * (-roll) + 0.0185* (-rollSpeed);

	//R=1300
//	nickRefTmp =       -0.0136* (xRef-x) +   -0.0376* (xSpeedRef-xSpeed) + -0.0006* (yRef-y) + -0.0015* (ySpeedRef-ySpeed) +  0.0536* (-nick) + 0.0456*  (-nickSpeed) + -0.0012* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp =  -0.0239* (xRef-x) +    -0.0657* (xSpeedRef-xSpeed) +  -0.0010* (yRef-y) + -0.0027* (ySpeedRef-ySpeed) +  0.0936* (-nick) + 0.0797*  (-nickSpeed) + -0.0021* (-roll) + 0.0001* (-rollSpeed);
//	rollRefTmp =      -0.0009* (xRef-x) +  -0.0028* (xSpeedRef-xSpeed) +  0.0200*  (yRef-y) +  0.0634*  (ySpeedRef-ySpeed) +  0.0038* (-nick) + -0.0003* (-nickSpeed) +  0.0500*  (-roll) +  0.0192* (-rollSpeed);
//	rollSpeedRefTmp = -0.0008* (xRef-x) +  -0.0027* (xSpeedRef-xSpeed) + 0.0189*  (yRef-y) + 0.0600*  (ySpeedRef-ySpeed) + 0.0036* (-nick) + -0.0003* (-nickSpeed) + 0.0473 * (-roll) + 0.0181* (-rollSpeed);

	//R=1500
//	nickRefTmp =       -0.0127* (xRef-x) +   -0.0361* (xSpeedRef-xSpeed) + -0.0005* (yRef-y) + -0.0015* (ySpeedRef-ySpeed) +  0.0516* (-nick) + 0.0440*  (-nickSpeed) + -0.0011* (-roll) + 0.0001* (-rollSpeed);
//	nickSpeedRefTmp =  -0.0222* (xRef-x) +    -0.0632* (xSpeedRef-xSpeed) +  -0.0009* (yRef-y) + -0.0026* (ySpeedRef-ySpeed) +  0.0902* (-nick) + 0.0769*  (-nickSpeed) + -0.0020* (-roll) + 0.0001* (-rollSpeed);
//	rollRefTmp =      -0.0008* (xRef-x) +  -0.0027* (xSpeedRef-xSpeed) +  0.0186*  (yRef-y) +  0.0610*  (ySpeedRef-ySpeed) +  0.0037* (-nick) + -0.0003* (-nickSpeed) +  0.0482*  (-roll) +  0.0185* (-rollSpeed);
//	rollSpeedRefTmp = -0.0008* (xRef-x) +  -0.0025* (xSpeedRef-xSpeed) + 0.0176*  (yRef-y) + 0.0577*  (ySpeedRef-ySpeed) + 0.0035* (-nick) + -0.0002* (-nickSpeed) + 0.0456 * (-roll) + 0.0175* (-rollSpeed);

	//end_L1



	//Saturation
	if(nickRefTmp>1)nickRefTmp= 1;
	else if(nickRefTmp<-1) nickRefTmp=-1;

	if(nickSpeedRefTmp>1) nickSpeedRefTmp=1;
	else if(nickSpeedRefTmp<-1) nickSpeedRefTmp=-1;

	if(rollRefTmp>1) rollRefTmp=1;
	else if(rollRefTmp<-1) rollRefTmp=-1;

	if(rollSpeedRefTmp>1) rollSpeedRefTmp=1;
	else if(rollSpeedRefTmp<-1) rollSpeedRefTmp=-1;


	//L2
	*nickRef = nickRefTmp * 0.5236;      //denormalize
	*nickSpeedRef = nickSpeedRefTmp * 3.1416;      //denormalize
	*rollRef = rollRefTmp * 0.5236;      //denormalize
	*rollSpeedRef = rollSpeedRefTmp * 3.1416;      //denormalize
	//end_L2



}

