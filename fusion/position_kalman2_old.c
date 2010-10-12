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
 *   @brief This is an example file
 *
 *   Multisensor Information Filter for Pixhawk Position Estimation
 *	 Note: this is not a KF, however it is derived based on KF for multisensor fusion!
 *
 *   @author Amirehsan Sarabadani Tafreshi <sarabada@student.ethz.ch>
 *   
 *
 */

/** @addtogroup code_examples */
/*@{*/
 
#include "position_kalman2.h"
#include <math.h>
#include <stdbool.h>
#define TS 0.005f

/*
void position_kalman_init(void)
{
	state.x = 0;
	state.y = 0;
	state.z = 0;
	state_dot.x = 0;
	state_dot.y = 0;
	state_dot.z = 0;
	state_ddot.x = 0;
	state_ddot.y = 0;
	state_ddot.z = 0;
	bias_ddot.x=global_data.param[PARAM_ACC_NAVI_OFFSET_X]/100;
	bias_ddot.y=global_data.param[PARAM_ACC_NAVI_OFFSET_Y]/100;
	bias_ddot.z=global_data.param[PARAM_ACC_NAVI_OFFSET_Z]/100;
}
*/
// Define Vision Input data Struct
//vision_t data;

static float_vect3 state={0,0,0};
static float_vect3 state_dot={0,0,0};
static float_vect3 state_ddot={0,0,0};
static float_vect3 bias_ddot={0,0,0};

void x_position_kalman2(float_vect3 acell, vision_t data, float_vect3* pos_est, float_vect3* vel_est,float_vect3* accel_est,float_vect3* bias_est)
{
    static float a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34, a41, a42, a43, a44; /* Forms Covariance matrix P_priori */
	static float p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34, p41, p42, p43, p44; /* Forms Covariance matrix P_posteriori */
	static float q11, q22, q33, q44, r11, r22; /* Q and R matrices */
	float det; /* det(denumerator) */
    static float x_est, xd_est, xdd_est, x_prior, xd_prior, xdd_prior, xbias_prior,test;

     
	static bool initialized = false;
     
    if (!initialized)
	{ /* Initialization of variables */
    	test=0.f;
		x_prior = 0;
		xd_prior = 0;
		xdd_prior = 0;
        xbias_prior=-0.25;
		x_est = 0;
		xd_est = 0;
		xdd_est = 0;
		state_ddot.x=acell.x;
		state_ddot.y=acell.y;
		state_ddot.z=acell.z;
		/* Parameters to tune( Uncertainity on the estimation(p), measurement(r) and model(q)). */
		p11 = 1;
		p12 = 0;
		p13 = 0;
        p14=0;
		p21 = 0;
		p22 = 0.2;
		p23 = 0;
        p24 = 0;
		p31 = 0;
		p32 = 0;
		p33 = 1;
        p34=0;
        p41=0;
        p42=0;
        p43=0;
        p44=0.1;
		r11 = 0.0001;
		r22 = 0.05;
		q11 =1.f * TS * TS * TS / 6.f;
		q22 = 1.f * TS * TS / 2.f;
		q33 = 1.f * TS;
        q44=0;
        initialized = true;
	}
    /* if needed for acceleration scaling */
/*
     accel.x=accel.x/100;
*/

	/* The Prediction Stage */
	x_prior = x_prior +state_dot.x * TS + state_ddot.x * TS * TS / 2;
	xd_prior = xd_prior + state_ddot.x * TS;
	xdd_prior=xdd_prior -bias_ddot.x;
	xbias_prior=xbias_prior;

	/* The Prediction stage Covariance */
	a11 = p11 + q11 + TS * p21 + TS*((p32 * TS * TS)/2 + p22 * TS + p12) + (TS * TS * p31)/2 + (TS * TS *((p33 * TS * TS)/2 + p23*TS + p13))/2;
	
	a12 = p12 + TS * p22 + TS *((p33 * TS * TS)/2 + p23 * TS + p13) + (TS * TS * p32)/2;

	a13 = p13 - p14 + TS * p23 - TS * p24 + (TS * TS * p33)/2 - (TS * TS * p34)/2;

	a14 = (p34 * TS * TS)/2 + p24 * TS + p14; 

	a21 = p21 + (TS * TS *(p23 + TS * p33))/2 + TS * p31 + TS *(p22 + TS * p32); 

	a22 = p22 + q22 + TS * p32 + TS *(p23 + TS * p33);

	a23 = p23 - p24 + TS * p33 - TS * p34;
	
	a24 = p24 + TS * p34;

	a31 = p31 - p41 + TS *(p32 - p42) + (TS * TS *(p33 - p43))/2;

	a32 = p32 - p42 + TS*(p33 - p43);

	a33 = p33 - p34 - p43 + p44 + q33;

	a34 = p34 - p44;

	a41 = (p43 * TS * TS)/2 + p42 * TS + p41;

	a42 = p42 + TS * p43;
	
	a43 = p43 - p44;

	a44 = p44 + q44;





	/* Vision available */
	/* Estimation Stage */
	if (data.new_data == 1)
	{
		/* a posteriori Covariance */

		det=1.f/(-a31 * a13 + a33 * a11 + r22 * a11 + r11 * a33 + r11 * r22);

		p11 = ((-a31 * a13 + r22 * a11 + a33 * a11) * r11) * det;

		p12 = ((-a32 * a13 + a33 * a12 + r22 * a12) * r11) * det;

		p13 = (a13 * r11 * r22) * det;

		p14 = (-(-a14 * a33 + a13 * a34 - r22 * a14) * r11) * det;

		p21 = ((a33 * a21 - a31 * a23 + r22 * a21) * r11) * det;

		p22 = (-a33 * a21 * a12 + a32 * a21 * a13 + a31 * a12 * a23 - a31 * a13 * a22 + a33 * a11 * a22

         - a32 * a11 * a23 + r22 * a11 * a22 - r22 * a21 * a12 + r11 * a22 * a33 - r11 * a32 * a23

         + a22 * r11 * r22) * det;

		p23 = ((r11 * a23 - a21 * a13 + a23 * a11) * r22) * det;

		p24 = - (-a23 * a31 * a14 - a21 * a13 * a34 + a24 * a31 * a13 + a21 * a14 * a33 + a11 * a23 * a34

         - a11 * a24 * a33 - r22 * a11 * a24 + r22 * a21 * a14 + r11 * a23 * a34 - r11 * a24 * a33

         - a24 * r11 * r22) * det;


		p31 = (a31 * r11 * r22) * det;

		p32 = ((r11 * a32 - a31 * a12 + a32 * a11) * r22) * det;

		p33 = ((-a31 * a13 + a33 * a11 + r11 * a33) * r22) * det;

		p34 = ((-a31 * a14 + a34 * a11 + r11 * a34) * r22) * det;

		p41 = -( r11 * (-a41 * a33 + a43 * a31 - r22 * a41)) * det;

		p42 = -(-a43 * a31 * a12 - a41 * a13 * a32 + a42 * a31 * a13 + a41 * a12 * a33 + a11 * a43 * a32

         - a11 * a42 * a33 - r22 * a42 * a11 + r22 * a41 * a12 + r11 * a43 * a32 - r11 * a42 * a33

         - a42 * r11 * r22) * det;

		p43 = ((-a41 * a13 + a43 * a11 + r11 * a43) * r22) * det;

		p44 = (-a44 * a31 * a13 - a41 * a14 * a33 + a43 * a31 * a14 + a41 * a13 * a34 + a11 * a44 * a33

         - a11 * a43 * a34 + r22 * a44 * a11 - r22 * a41 * a14 + r11 * a44 * a33 - r11 * a43 * a34

         + a44 * r11 * r22) * det;

		/* Estimation stage */

		state.x = x_prior + ((-a31 * a13 + r22 * a11 + a33 * a11) * (data.pos.x - x_prior) + a13 * r11 * (acell.x - xdd_prior)) * det;
		state_dot.x = xd_prior + ((a33 * a21 - a31 * a23 + r22 * a21) * (data.pos.x - x_prior) + (r11 * a23 - a21 * a13 + a23 * a11) * (acell.x - xdd_prior)) * det ;
		state_ddot.x = xdd_prior + (a31 * r22 * (data.pos.x - x_prior) + (-a31 * a13 + a33 * a11 + r11 * a33) * (acell.x - xdd_prior)) * det;
		bias_ddot.x = xbias_prior + (-(-a41 * a33 + a43 * a31 - r22 * a41) * (data.pos.x - x_prior) + (-a41 * a13 + a43 * a11 + r11 * a43) * (acell.x - xdd_prior)) * det;
		x_prior=state.x;
		xd_prior=state_dot.x;
		xdd_prior=state_ddot.x;
		xbias_prior=bias_ddot.x;
test=1;
	}
	/* just IMU available */
	/* Estimation Stage */
	if (test == 0.f )

	{
	/* a posteriori Covariance */

		det=1.f/(a33 + r22);

		p11 = (-a31 * a13 + r22 * a11 + a33 * a11) * det;

		p12 = (-a32 * a13 + a33 * a12 + r22 * a12) * det;

		p13 = (a13 * r22) * det;

		p14 = -(-a14 * a33 + a13 * a34 - r22 * a14) * det;

		p21 = (a33 * a21 - a31 * a23 + r22 * a21) * det;

		p22 = (a33 * a22 - a32 * a23 + r22 * a22) * det;

		p23 = (a23 * r22) * det;

		p24 = -(a23 * a34 - a24 * a33 - r22 * a24) * det;

		p31 = (a31 * r22) * det;

		p32 = (a32 * r22) * det;

		p33 = (a33 * r22) * det;

		p34 = (a34 * r22) * det;

		p41 = -( -a41 * a33 + a43 * a31 - r22 * a41) * det;

		p42 = -(a43 * a32 - a42 * a33 - r22 * a42) * det;

		p43 = (a43 * r22) * det;

		p44 = (a44 * a33 - a43 * a34 + r22 * a44) * det;

		/* Estimation stage */

		state.x =x_prior + ( a13 * (acell.x - xdd_prior)) * det;
		state_dot.x =xd_prior + ( a23 * (acell.x - xdd_prior)) * det ;
		state_ddot.x=xdd_prior + ( a33 * (acell.x - xdd_prior)) * det;
		bias_ddot.x = xbias_prior + ( a43 * (acell.x - xdd_prior)) * det;
		x_prior=state.x;
		xd_prior=state_dot.x;
		xdd_prior=state_ddot.x;
		xbias_prior=bias_ddot.x;

	}
                               

// Estimations
pos_est->x = x_prior; // position estimation
vel_est->x = xd_prior; // velocity estimation
accel_est->x = xdd_prior; // Acceleration estimation
bias_est->x=xbias_prior;
data.new_data=0;
test=0.f;
	
}
void y_position_kalman2(float_vect3 acell, vision_t data, float_vect3* pos_est, float_vect3* vel_est)
{
    static float a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34, a41, a42, a43, a44; /* Forms Covariance matrix P_priori */
	static float p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34, p41, p42, p43, p44; /* Forms Covariance matrix P_posteriori */
	static float q11, q22, q33, q44, r11, r22; /* Q and R matrices */
	float det; /* det(denumerator) */
    static float y_est, yd_est, ydd_est, y_prior, yd_prior, ydd_prior, ybias_prior;
     
    static bool initialized = false;

     if (!initialized)
	{ /* Initialization of variables */
/* */
		y_prior = 0;
		yd_prior = 0;
		ydd_prior = 0;
        ybias_prior=0;
		y_est = 0;
		yd_est = 0;
		ydd_est = 0;
		/* Parameters to tune( Uncertainity on the estimation(p), measurement(r) and model(q)). */
		p11 = 0.2;
		p12 = 0;
		p13 = 0;
        p14=0;
		p21 = 0;
		p22 = 5;
		p23 = 0;
        p24 = 0;
		p31 = 0;
		p32 = 0;
		p33 = 1;
        p34=0;
        p41=0;
        p42=0;
        p43=0;
        p44=0;
		r11 = 0.001;
		r22 = 0.0001;
		q11 = 1 * TS * TS * TS / 6;
		q22 = 0.1 * TS * TS / 2;
		q33 = 1 * TS;
        q44=0;
        initialized = true;
	}
    /* if needed for acceleration scaling */
/*
     accel.y=accel.y/100;
*/

	/* The Prediction Stage */
	y_prior = state.y + state_dot.y * TS + state_ddot.y * TS * TS / 2;
	yd_prior = state_dot.y + state_ddot.y * TS;
	ydd_prior = state_ddot.y-bias_ddot.y;
	ybias_prior=bias_ddot.y;

	/* The Prediction stage Covariance */
	a11 = p11 + q11 + TS * p21 + TS*((p32 * TS * TS)/2 + p22 * TS + p12) + (TS * TS * p31)/2 + (TS * TS *((p33 * TS * TS)/2 + p23*TS + p13))/2;
	
	a12 = p12 + TS * p22 + TS *((p33 * TS * TS)/2 + p23 * TS + p13) + (TS * TS * p32)/2;

	a13 = p13 - p14 + TS * p23 - TS * p24 + (TS * TS * p33)/2 - (TS * TS * p34)/2;

	a14 = (p34 * TS * TS)/2 + p24 * TS + p14; 

	a21 = p21 + (TS * TS *(p23 + TS * p33))/2 + TS * p31 + TS *(p22 + TS * p32); 

	a22 = p22 + q22 + TS * p32 + TS *(p23 + TS * p33);

	a23 = p23 - p24 + TS * p33 - TS * p34;
	
	a24 = p24 + TS * p34;

	a31 = p31 - p41 + TS *(p32 - p42) + (TS * TS *(p33 - p43))/2;

	a32 = p32 - p42 + TS*(p33 - p43);

	a33 = p33 - p34 - p43 + p44 + q33;

	a34 = p34 - p44;

	a41 = (p43 * TS * TS)/2 + p42 * TS + p41;

	a42 = p42 + TS * p43;
	
	a43 = p43 - p44;

	a44 = p44 + q44;





	/* Vision available */
	/* Estimation Stage */
	if (data.new_data == 1)
	{
		/* a posteriori Covariance */

		det=1.f/(-a31 * a13 + a33 * a11 + r22 * a11 + r11 * a33 + r11 * r22);

		p11 = ((-a31 * a13 + r22 * a11 + a33 * a11) * r11) * det;
		
		p12 = ((-a32 * a13 + a33 * a12 + r22 * a12) * r11) * det;

		p13 = (a13 * r11 * r22) * det;

		p14 = (-(-a14 * a33 + a13 * a34 - r22 * a14) * r11) * det;

		p21 = ((a33 * a21 - a31 * a23 + r22 * a21) * r11) * det;
		
		p22 = (-a33 * a21 * a12 + a32 * a21 * a13 + a31 * a12 * a23 - a31 * a13 * a22 + a33 * a11 * a22

         - a32 * a11 * a23 + r22 * a11 * a22 - r22 * a21 * a12 + r11 * a22 * a33 - r11 * a32 * a23

         + a22 * r11 * r22) * det;

		p23 = ((r11 * a23 - a21 * a13 + a23 * a11) * r22) * det;

		p24 = - (-a23 * a31 * a14 - a21 * a13 * a34 + a24 * a31 * a13 + a21 * a14 * a33 + a11 * a23 * a34

         - a11 * a24 * a33 - r22 * a11 * a24 + r22 * a21 * a14 + r11 * a23 * a34 - r11 * a24 * a33

         - a24 * r11 * r22) * det;
 

		p31 = (a31 * r11 * r22) * det;

		p32 = ((r11 * a32 - a31 * a12 + a32 * a11) * r22) * det;

		p33 = ((-a31 * a13 + a33 * a11 + r11 * a33) * r22) * det;

		p34 = ((-a31 * a14 + a34 * a11 + r11 * a34) * r22) * det;

		p41 = -( r11 * (-a41 * a33 + a43 * a31 - r22 * a41)) * det;

		p42 = -(-a43 * a31 * a12 - a41 * a13 * a32 + a42 * a31 * a13 + a41 * a12 * a33 + a11 * a43 * a32

         - a11 * a42 * a33 - r22 * a42 * a11 + r22 * a41 * a12 + r11 * a43 * a32 - r11 * a42 * a33

         - a42 * r11 * r22) * det;

		p43 = ((-a41 * a13 + a43 * a11 + r11 * a43) * r22) * det;

		p44 = (-a44 * a31 * a13 - a41 * a14 * a33 + a43 * a31 * a14 + a41 * a13 * a34 + a11 * a44 * a33

         - a11 * a43 * a34 + r22 * a44 * a11 - r22 * a41 * a14 + r11 * a44 * a33 - r11 * a43 * a34

         + a44 * r11 * r22) * det;

		/* Estimation stage */

		state.y = y_prior + ((-a31 * a13 + r22 * a11 + a33 * a11) * (data.pos.y - y_prior) + a13 * r11 * (acell.y - ydd_prior)) * det;
		state_dot.y = yd_prior + ((a33 * a21 - a31 * a23 + r22 * a21) * (data.pos.y - y_prior) + (r11 * a23 - a21 * a13 + a23 * a11) * (acell.y - ydd_prior)) * det ;
		state_ddot.y = ydd_prior + (a31 * r22 * (data.pos.y - y_prior) + (-a31 * a13 + a33 * a11 + r11 * a33) * (acell.y - ydd_prior)) * det;
		bias_ddot.y = ybias_prior + (-(-a41 * a33 + a43 * a31 - r22 * a41) * (data.pos.y - y_prior) + (-a41 * a13 + a43 * a11 + r11 * a43) * (acell.y - ydd_prior)) * det;
		
	}
	/* just IMU available */
	/* Estimation Stage */
	if (data.new_data == 0)
	{
	/* a posteriori Covariance */

		det=1.f/(a33 + r22);

		p11 = (-a31 * a13 + r22 * a11 + a33 * a11) * det;
		
		p12 = (-a32 * a13 + a33 * a12 + r22 * a12) * det;

		p13 = (a13 * r22) * det;

		p14 = -(-a14 * a33 + a13 * a34 - r22 * a14) * det;

		p21 = (a33 * a21 - a31 * a23 + r22 * a21) * det;
		
		p22 = (a33 * a22 - a32 * a23 + r22 * a22) * det;

		p23 = (a23 * r22) * det;

		p24 = -(a23 * a34 - a24 * a33 - r22 * a24) * det;
 
		p31 = (a31 * r22) * det;

		p32 = (a32 * r22) * det;

		p33 = (a33 * r22) * det;

		p34 = (a34 * r22) * det;

		p41 = -( -a41 * a33 + a43 * a31 - r22 * a41) * det;

		p42 = -(a43 * a32 - a42 * a33 - r22 * a42) * det;

		p43 = (a43 * r22) * det;

		p44 = (a44 * a33 - a43 * a34 + r22 * a44) * det;

		/* Estimation stage */

		state.y = y_prior + ( a13 * (acell.y - ydd_prior)) * det;
		state_dot.y = yd_prior + ( a23 * (acell.y - ydd_prior)) * det ;
		state_ddot.y = ydd_prior + ( a33 * (acell.y - ydd_prior)) * det;
		bias_ddot.y = ybias_prior + ( a43 * (acell.y - ydd_prior)) * det;
		
	}
                               

// Estimations
pos_est->y = state.y; // position estimation
vel_est->y = state_dot.y; // velocity estimation
//accel_est->y = state_ddot.y; // Acceleration estimation
data.new_data=0;
	
}
