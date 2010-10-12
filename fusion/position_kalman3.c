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
 *   @brief Vision Only Kalman Filter for Pixhawk Position Estimation, ver2
 *	 
 *   @author Amirehsan Sarabadani Tafreshi <sarabada@student.ethz.ch>
 *   
 */

/** @addtogroup fusion */
/*@{*/
 
#include "position_kalman3.h"
#include <math.h>
#include <stdbool.h>

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
	state3_ddot.z = 0;
	bias_ddot.x=global_data.param[PARAM_ACC_NAVI_OFFSET_X]/100;
	bias_ddot.y=global_data.param[PARAM_ACC_NAVI_OFFSET_Y]/100;
	bias_ddot.z=global_data.param[PARAM_ACC_NAVI_OFFSET_Z]/100;
}
*/
// Define Vision Input data Struct
//vision_t data;

static float_vect3 state3={0,0,0};
static float_vect3 state3_dot={0,0,0};
float maxV=1.5,minV=-1.5,thresh=0.5,slope=0.6;

void position_kalman3_init (float_vect3* pos_est, float_vect3* vel_est)
{
state3=*pos_est;
state3_dot=*vel_est;

}

void x_position_kalman3(vision_t* data, float_vect3* pos_est, float_vect3* vel_est)
{
    static float a11,a12,a21,a22; /* Forms Covariance matrix P_priori */
	static float p11, p12, p21, p22; /* Forms Covariance matrix P_posteriori */
	static float q11, q12, q21, q22, r11,xprior,xdotprior, counter;//old,oldtime; /* Q and R matrices */
	float det,T = 0.005f,time=0;//Tav; /* det(denumerator) */
         
	static bool initialized = false;
     
    if (!initialized)
	{ /* Initialization of variables */
		/* Parameters to tune( Uncertainity on the estimation(p), measurement(r) and model(q)). */
		p11 = 1;
		p12 = 0;
		p21 = 0;
		p22 = 1;
		r11 = 0.0009f;
		q11 = 0.000001f;//1.f * TS * TS * TS / 6.f;
		q12=0*T*T*T*T;
		q21=0*T*T*T*T;
		counter =0;
//		old=0;
//		oldtime=0;
//		q21=2;
		xprior=0;
		xdotprior=0;
		q22 = 150.0f*T*T;//0.5f * TS * TS / 2.f;
		initialized = true;
	}
	/* timer to stop integration */
	counter+=1;
	time=counter*T;

	/* The Prediction Stage */

	xprior = state3.x + state3_dot.x * T;
	if (time>thresh)
	{
		xdotprior=state3_dot.x*slope;
	}
	else
	{
	xdotprior = state3_dot.x;
	}
	/* The Prediction stage Covariance */
	a11 = p11 + T * p21 + (p12 + T * p22) * T + q11;
	
	a12 = p12 + T * p22+ q12;

	a21 = p21 + T * p22+ q21;

	a22 = p22 + q22;


	if (data->new_data == 1)
	{
	/* Estimation Stage */
	det = 1.0f/(a11 + r11);
	state3.x = xprior+ a11* (data->pos.x - xprior) * det;
	state3_dot.x= xdotprior+ a21 * (data->pos.x - xprior) * det;
	/* limit velocity */
	if (state3_dot.x>maxV)
	{
		state3_dot.x = maxV;
	}
	if (state3_dot.x<minV)
	{
		state3_dot.x = minV;
	}

//	Tav=(data->comp_end-oldtime)/1000000.f;
//	state3_dot.x=(state3.x-old)/Tav;
	xprior=state3.x;
	xdotprior=state3_dot.x;
//	old=xprior;
//	oldtime=data->comp_end;
	/* a posteriori estimate covariance */
	p11= (1.f-a11*det)*a11;
	p12= (1.f-a11*det)*a12;
	p21= a21-(a21*a11)*det;
	p22= a22-(a21*a12)*det;
	counter=0;
	}
	else
	{
	state3.x = xprior;
	state3_dot.x = xdotprior;
	p11 = a11;
	p12 = a12;
	p21 = a21;
	p22 = a22;
	}
	pos_est->x=xprior;
	vel_est->x=xdotprior;
//	data->new_data = 0;
}

void y_position_kalman3(vision_t* data, float_vect3* pos_est, float_vect3* vel_est)
{
    static float a11,a12,a21,a22; /* Forms Covariance matrix P_priori */
	static float p11, p12, p21, p22; /* Forms Covariance matrix P_posteriori */
	static float q11, q12, q21, q22, r11,yprior,ydotprior,counter; /* Q and R matrices */
	float det,T=0.005f,time=0; /* det(denumerator) */
         
	static bool initialized = false;
     
    if (!initialized)
	{ /* Initialization of variables */
		/* Parameters to tune( Uncertainity on the estimation(p), measurement(r) and model(q)). */
		p11 = 1;
		p12 = 0;
		p21 = 0;
		p22 = 1;
		r11 = 0.0009;
		q11 = 0.000001f;//1.f * TS * TS * TS / 6.f;
		q12=0.f;
		q21=0.0f;
		yprior=0;
		ydotprior=0;
		q22 = 150.0f*T*T ;//0.5f * TS * TS / 2.f;
		initialized = true;
	}

	/* timer to stop integration */
	counter+=1;
	time=counter*T;
    /* The Prediction Stage */
	yprior = state3.y + state3_dot.y * T;
	if (time>thresh)
	{
		ydotprior=state3_dot.y*slope;
	}
	else
	{
	ydotprior = state3_dot.y;
	}
	/* The Prediction stage Covariance */
	a11 = p11 + T * p21 + (p12 + T * p22) * T + q11;
	
	a12 = p12 + T * p22+ q12;

	a21 = p21 + T * p22+ q21;

	a22 = p22 + q22;

	if (data->new_data == 1)
	{
	/* Estimation Stage */
	det = 1.0f/(a11 + r11);
	state3.y = yprior+ a11* (data->pos.y - yprior) * det;
	state3_dot.y= ydotprior+ a21 * (data->pos.y - yprior) * det;
	/* limit velocity */
	if (state3_dot.y>maxV)
	{
			state3_dot.y = maxV;
	}
	if (state3_dot.y<minV)
	{
			state3_dot.y = minV;
	}
	yprior=state3.y;
	ydotprior=state3_dot.y;

	/* a posteriori estimate covariance */
	p11= (1.f-a11*det)*a11;
	p12= (1.f-a11*det)*a12;
	p21= a21-(a21*a11)*det;
	p22= a22-(a21*a12)*det;
	counter=0;
	}
	else
	{
	state3.y = yprior;
	state3_dot.y = ydotprior;
	p11 = a11;
	p12 = a12;
	p21 = a21;
	p22 = a22;
	}
	pos_est->y=yprior;
	vel_est->y=ydotprior;
//	data->new_data = 0;
}
void z_position_kalman3(vision_t* data, float_vect3* pos_est, float_vect3* vel_est)
{
    static float a11,a12,a21,a22; /* Forms Covariance matrix P_priori */
	static float p11, p12, p21, p22; /* Forms Covariance matrix P_posteriori */
	static float q11, q12, q21, q22, r11,zprior,zdotprior, counter; /* Q and R matrices */
	float det,T=0.005f, time=0; /* det(denumerator) */
         
	static bool initialized = false;
     
    if (!initialized)
	{ /* Initialization of variables */
		/* Parameters to tune( Uncertainity on the estimation(p), measurement(r) and model(q)). */
		p11 = 1;
		p12 = 0;
		p21 = 0;
		p22 = 1;
		r11 = 0.0009;
		q11 = 0.000001f;//1.f * TS * TS * TS / 6.f;
		q12=0.0f;
		q21=0.0f;
		zprior=0;
		zdotprior=0;
		q22 = 150.0f*T*T;//0.5f * TS * TS / 2.f;
		initialized = true;
	}
	/* timer to stop integration */
	counter+=1;
	time=counter*T;
    /* The Prediction Stage */
	zprior = state3.z + state3_dot.z * T;
	if (time>thresh)
	{
		zdotprior=state3_dot.z*slope;
	}
	else
	{
	zdotprior = state3_dot.z;
	}
	/* The Prediction stage Covariance */
	a11 = p11 + T * p21 + (p12 + T * p22) * T + q11;
	
	a12 = p12 + T * p22+ q12;

	a21 = p21 + T * p22+ q21;

	a22 = p22 + q22;

	if (data->new_data == 1)
	{
	/* Estimation Stage */
	det = 1.0f/(a11 + r11);
	state3.z = zprior+ a11* (data->pos.z - zprior) * det;
	state3_dot.z= zdotprior+ a21 * (data->pos.z - zprior) * det;
	/* limit velocity */
	if (state3_dot.z>maxV)
	{
		state3_dot.z = maxV;
	}
	if (state3_dot.z<minV)
	{
		state3_dot.z = minV;
	}

	zprior=state3.z;
	zdotprior=state3_dot.z;

	/* a posteriori estimate covariance */
	p11= (1.f-a11*det)*a11;
	p12= (1.f-a11*det)*a12;
	p21= a21-(a21*a11)*det;
	p22= a22-(a21*a12)*det;
	counter=0;
	}
	else
	{
	state3.z = zprior;
	state3_dot.z = zdotprior;
	p11 = a11;
	p12 = a12;
	p21 = a21;
	p22 = a22;
	}
	pos_est->z=zprior;
	vel_est->z=zdotprior;
//	data->new_data = 0;
}

/*@}*/
