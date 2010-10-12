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
 *   This is the more detailed description of this file.
 *
 *   @author Andrea Cortinovis <candrea@student.ethz.ch>
 *   @author Amir <sarab@student.ethz.ch>
 *
 */

/** @addtogroup code_examples */
/*@{*/

#include "position_kalman.h"
#include "global_data.h"
#include <math.h>
#define TS 0.005f
//#define DAMP 0.90f

/** @brief Function that does nothing */
/*void position_kalman(float xACC,float yACC,float zACC,float rollSpeed,float pitchSpeed,float yawSpeed,float xMag,float yMag,float  zMag,float pressure,float distance,float* roll,float* pitch,float* yaw,float* alt,float* groundDistance)
 */

/* Define structs for x, x_d and x_dd (last states)*/
float_vect3 state;
float_vect3 state_dot;
float_vect3 state_ddot;
/* Define Vision Input data Struct */
//vision data;

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
}

float xprev, yprev, zprev, tprev, Delta_T;
int xN = 1;
void position_kalman(float_vect3* acell, vision_t* data, float_vect3* pos_est,
		float_vect3* vel_est)
{

	state_ddot.x = (acell->x - global_data.param[PARAM_ACC_NAVI_OFFSET_X]) / 100;
	state_ddot.y = (acell->y - global_data.param[PARAM_ACC_NAVI_OFFSET_Y]) / 100;
	state_ddot.z = (acell->z - global_data.param[PARAM_ACC_NAVI_OFFSET_Z]) / 100;

	if (global_data.param[PARAM_SEND_SLOT_DEBUG_6] == 0)
	{
		if (data->new_data == 1)
		{
			Delta_T = (data->comp_end - tprev) / 1000000;

			xprev = state.x;
			yprev = state.y;
			zprev = state.z;
			tprev = data->comp_end;

			state.x = data->pos.x;
			state.y = data->pos.y;
			state.z = data->pos.z;
			//Don't update velocity from vision
			//		state_dot.x = (state.x - xprev) / Delta_T;
			//		state_dot.y = (state.y - yprev) / Delta_T;
			//		state_dot.z = (state.z - zprev) / Delta_T;

			//		state_dot.x = state_dot.x + TS * state_ddot.x;
			//		state_dot.y = state_dot.y + TS * state_ddot.y;
			//		state_dot.z = state_dot.z + TS * state_ddot.z;

			data->new_data = 0;
		}
		else
		{
			if (global_data.param[PARAM_SEND_SLOT_DEBUG_6] == 0)
			{
				state_dot.x += TS * state_ddot.x;
				state_dot.y += TS * state_ddot.y;
				state_dot.z += TS * state_ddot.z;
//ignore acc because of offset
				state.x += (state_dot.x-global_data.param[PARAM_VEL_OFFSET_X]) * TS;// + state_ddot.x * TS * TS / 2;
				state.y += (state_dot.y-global_data.param[PARAM_VEL_OFFSET_Y]) * TS;// + state_ddot.y * TS * TS / 2;
				state.z += (state_dot.z-global_data.param[PARAM_VEL_OFFSET_Z]) * TS;// + state_ddot.z * TS * TS / 2;
			}
		}
		//reduce impact of offset
		// TODO implement low-pass filter on transformed accelerometer data and update the offset.
		// only works if we are not to fast moving
		state_dot.x *= global_data.param[PARAM_VEL_DAMP];
		state_dot.y *= global_data.param[PARAM_VEL_DAMP];
		state_dot.z *= global_data.param[PARAM_VEL_DAMP];

		/*state.z=state_vis_pos.z; */

		xN = 1;
	}
	else
	{

		//TESTING KALMAN ON X
		if (data->new_data == 1)

		{

			yprev = state.y;
			zprev = state.z;
			state.y = data->pos.y;
			state.z = data->pos.z;
			}
		state_dot.y *= global_data.param[PARAM_VEL_DAMP];
		state_dot.z *= global_data.param[PARAM_VEL_DAMP];
		x_position_kalman(*acell, *data, pos_est, xN);
		xN = 0;
	}

	pos_est->x = state.x;
	pos_est->y = state.y;
	pos_est->z = state.z;
	/*	*y=xACC*yACC; */
	/*	*z=xACC*yACC;*/
	vel_est->x = state_dot.x;
	vel_est->y = state_dot.y;
	vel_est->z = state_dot.z;
}

/* Hallo Laurens, das isch mini funktion. Wie du gseit hesch.. eifach agh'ngt.. ich has aber nid gschafft
 * das gange uszkommentiere.. lool, falls frooge sin chasch mr eifach aluette oder e email schriibe.
 * Das wär text markiere und  Ctrl+/ ;-) */

//
///* Define structs for x, x_d and x_dd (last states) */
//float_vect3 state;
//float_vect3 state_dot;
//float_vect3 state_ddot;
///* Define Vision Input data Struct */
//vision data;
//
//
void x_position_kalman(float_vect3 acell, vision_t data, float_vect3* pos_est,
		int N)
{
	//     static float Delta_T;
	static float a11, a12, a13, a21, a22, a23, a31, a32, a33; /* Forms Covariance matrix P_priori */
	static float p11, p12, p13, p21, p22, p23, p31, p32, p33; /* Forms Covariance matrix P_posteriori */
	static float q11, q22, q33, r11, r22; /* Q and R matrices */
	float s11, s12, s21, s22, det; /* Forms Residual Covariance matrix S */
	float k11, k12, k21, k22, k31, k32, k1, k2, k3; /* Forms Kalman Gain matrix K */
	static float x_est, xd_est, xdd_est, x_prior, xd_prior, xdd_prior;

	if (N == 1)
	{ /* Initialization of variables */
		/* move to function position_kalman_init() */
		//                float_vect3 state={0,0,0};
		//                float_vect3 state_dot={0,0,0};
		//                float_vect3 state_ddot={0,0,0};
		x_prior = 0;
		xd_prior = 0;
		xdd_prior = 0;
		x_est = 0;
		xd_est = 0;
		xdd_est = 0;
		/* Here covariances and variances have to be tuned. */
		p11 = 100;
		p12 = 0;
		p13 = 0;
		p21 = 0;
		p22 = 100;
		p23 = 0;
		p31 = 0;
		p32 = 0;
		p33 = 100;
		r11 = 0.01;
		r22 = 1;
		q11 = 1000 * TS * TS * TS / 6;
		q22 = 1000 * TS * TS / 2;
		q33 = 1000 * TS;
	}
	/* Reset Covariance... mhh, this is a problem Im working at..
	 I'm not sure why this happens... update as soon as possible.. */
	if (p11 + p22 + p33 > 10000)
	{
		p11 = 100;
		p12 = 0;
		p13 = 0;
		p21 = 0;
		p22 = 100;
		p23 = 0;
		p31 = 0;
		p32 = 0;
		p33 = 100;
	}

	/* Prior state update */
	x_prior = state.x + state_dot.x * TS + state_ddot.x * TS * TS / 2;
	state.y = state.y + state_dot.y * TS + state_ddot.y * TS * TS / 2;
	state.z = state.z + state_dot.z * TS + state_ddot.z * TS * TS / 2;
	xd_prior = state_dot.x + TS * state_ddot.x;
	state_dot.y = state_dot.y + TS * state_ddot.y;
	state_dot.z = state_dot.z + TS * state_ddot.z;
	xdd_prior = state_ddot.x;

	/* Prior Covariance */
	a11 = p11 + TS * p21 + TS * TS / 2 * p31 + TS * (p12 + TS * p22 + TS * TS
			/ 2 * p32) + TS * TS / 2 * (p13 + TS * p23 + TS * TS / 2 * p33)
			+ q11;
	a12 = p12 + TS * p22 + TS * TS / 2 * p32 + TS * (p13 + TS * p23 + TS * TS
			/ 2 * p33);
	a13 = p13 + p23 * TS + TS * TS / 2 * p33;

	a21 = p21 + TS * p31 + TS * (p22 + TS * p32) + TS * TS / 2 * (p23 + TS
			* p33);
	a22 = p22 + TS * p32 + TS * (p23 + TS * p33) + q22;
	a23 = p23 + TS * p33;

	a31 = p31 + TS * p32 + TS * TS / 2 * p33;
	a32 = p32 + TS * p33;
	a33 = p33 + q33;

	/* Vision available */
	if (data.new_data == 1)
	{
		//                                    Delta_T=data.comp_end-data.time_captured;

//		Delta_T = (data.comp_end - tprev) / 1000000;

		//         		xprev = state.x;
		//         		yprev = state.y;
		//         		zprev = state.z;
		tprev = data.comp_end;
		/* CONFIDENCE DEPENDENCY: INSERT HERE
		 That's the point to take consideration of the confidence factor,
		 it is stored in data.confidence and should be used to
		 change the weight of r11 (x postion from vision).
		 Alternatively you can also set k11, k21, k31 to zero..
		 This way you ommit completely the vision part and the algorithm is running
		 like the IMU-only case */
		/* Residual Covariance */
		det = (a11 + r11) * (a33 + r22) - a13 * a31;
		s11 = (a33 + r22) / det;
		s12 = -a13 / det;
		s21 = -a31 / det;
		s22 = (a11 + r11) / det;
		/* Kalman gain, K 2x3 */
		k11 = a11 * s11 + a13 * s21;
		k12 = a11 * s12 + a13 * s22;
		k21 = a21 * s11 + a23 * s21;
		k22 = a21 * s12 + a23 * s22;
		k31 = a31 * s11 + a33 * s21;
		k32 = a31 * s12 + a33 * s22;

		/* a posteriori state update */
		x_est = x_prior + (k11 * (data.pos.x - x_prior) + k12 * (acell.x
				- xdd_prior));
		xd_est = xd_prior + (k21 * (data.pos.x - x_prior) + k22 * (acell.x
				- xdd_prior));
		xdd_est = xdd_prior + (k31 * (data.pos.x - x_prior) + k32 * (acell.x
				- xdd_prior));

		/* a posteriori Covariance */
		p11 = a11 - (k11 * (k11 * s11 + k12 * s21) + k12 * (k11 * s12 + k12
				* s22));
		p12 = a12 - (k21 * (k11 * s11 + k12 * s21) + k22 * (k11 * s12 + k12
				* s22));
		p13 = a13 - (k31 * (k11 * s11 + k12 * s21) + k32 * (k11 * s12 + k12
				* s22));

		p21 = a21 - (k11 * (k21 * s11 + k22 * s21) + k12 * (k21 * s12 + k22
				* s22));
		p22 = a22 - (k21 * (k21 * s11 + k22 * s21) + k22 * (k21 * s12 + k22
				* s22));
		p23 = a23 - (k31 * (k21 * s11 + k22 * s21) + k32 * (k21 * s12 + k22
				* s22));

		p31 = a31 - (k11 * (k31 * s11 + k32 * s21) + k12 * (k31 * s12 + k32
				* s22));
		p32 = a32 - (k21 * (k31 * s11 + k32 * s21) + k22 * (k31 * s12 + k32
				* s22));
		p33 = a33 - (k31 * (k31 * s11 + k32 * s21) + k32 * (k31 * s12 + k32
				* s22));

		state.x = x_est;
		state_dot.x = xd_est;
		state_ddot.x = xdd_est;

		/* printf("X=%6.3f\n",state.x); */

	}
	/* just IMU available */
	if (data.new_data == 0)
	{
		/* Kalman Gain, K 3x1 */
		k1 = a13 / (a33 + r22);
		k2 = a23 / (a33 + r22);
		k3 = a33 / (a33 + r22);

		/* a posteriori state update */
		x_est = x_prior + k1 * (acell.x - xdd_prior);
		xd_est = xd_prior + k2 * (acell.x - xdd_prior);
		xdd_est = xdd_prior + k3 * (acell.x - xdd_prior);

		/* a posteriori Covariance */
		p11 = a11 - k1 * k1 / (a33 + r22);
		p12 = a12 - k1 * k2 / (a33 + r22);
		p13 = a13 - k1 * k3 / (a33 + r22);

		p21 = a21 - k2 * k1 / (a33 + r22);
		p22 = a22 - k2 * k2 / (a33 + r22);
		p23 = a23 - k2 * k3 / (a33 + r22);

		p31 = a31 - k3 * k1 / (a33 + r22);
		p32 = a32 - k3 * k2 / (a33 + r22);
		p33 = a33 - k3 * k3 / (a33 + r22);
		state.x = x_est;
		state_dot.x = xd_est;
		state_ddot.x = xdd_est;
	}

	pos_est->x = state.x;

	data.new_data = 0;

}
//void y_position_kalman(float_vect3 acell, vision data, float_vect3* pos_est,int N)
//{
//     static float Delta_T;
//     static float a11, a12, a13, a21, a22, a23, a31, a32, a33;     /* Forms Covariance matrix P_priori */
//     static float p11, p12, p13, p21, p22, p23, p31, p32, p33;     /* Forms Covariance matrix P_posteriori */
//     static float q11, q22, q33, r11, r22;                         /* Q and R matrices */
//     float s11, s12, s21, s22, det;                         /* Forms Residual Covariance matrix S */
//     float k11, k12, k21, k22, k31, k32, k1, k2, k3;        /* Forms Kalman Gain matrix K */
//     static float y_est, yd_est, ydd_est, y_prior, yd_prior, ydd_prior;
//
//     if (N==1){ /* Initialization of variables */
//                /* move to function position_kalman_init() */
//                float_vect3 state={0,0,0};
//                float_vect3 state_dot={0,0,0};
//                float_vect3 state_ddot={0,0,0};
//                y_prior=0;
//                yd_prior=0;
//                ydd_prior=0;
//                y_est=0;
//                yd_est=0;
//                ydd_est=0;
//                /* Here covariances and variances have to be tuned. */
//                p11=100;
//                p12=0;
//                p13=0;
//                p21=0;
//                p22=100;
//                p23=0;
//                p31=0;
//                p32=0;
//                p33=100;
//                r11=0.01;
//                r22=1;
//                q11=1000*TS*TS*TS/6;
//                q22=1000*TS*TS/2;
//                q33=1000*TS;
//               }
//               if (p11+p22+p33>10000){
//               p11=100;
//                p12=0;
//                p13=0;
//                p21=0;
//                p22=100;
//                p23=0;
//                p31=0;
//                p32=0;
//                p33=100;}
//
//             /* Prior state update */
//             y_prior=state.y+state_dot.y*TS+state_ddot.y*TS*TS/2;
//             yd_prior=state_dot.y+TS*state_ddot.y;
//             ydd_prior=state_ddot.y;
//
//             /* Prior Covariance */
//             a11=p11+TS*p21+TS*TS/2*p31+TS*(p12+TS*p22+TS*TS/2*p32)+TS*TS/2*(p13+TS*p23+TS*TS/2*p33)+q11;
//             a12=p12+TS*p22+TS*TS/2*p32+TS*(p13+TS*p23+TS*TS/2*p33);
//             a13=p13+p23*TS+TS*TS/2*p33;
//
//             a21=p21+TS*p31+TS*(p22+TS*p32)+TS*TS/2*(p23+TS*p33);
//             a22=p22+TS*p32+TS*(p23+TS*p33)+q22;
//             a23=p23+TS*p33;
//
//             a31=p31+TS*p32+TS*TS/2*p33;
//             a32=p32+TS*p33;
//             a33=p33+q33;
//
//             /* Vision available */
//             if (data.new_data==1){
//                                    Delta_T=data.comp_end-data.time_captured;
//
//                                    /* CONFIDENCE DEPENDENCY: INSERT HERE */
//
//                                    /* Residual Covariance */
//                                    det=(a11+r11)*(a33+r22)-a13*a31;
//                                    s11=(a33+r22)/det;
//                                    s12=-a13/det;
//                                    s21=-a31/det;
//                                    s22=(a11+r11)/det;
//                                    /* Kalman gain, K 2x3 */
//                                    k11=a11*s11+a13*s21;
//                                    k12=a11*s12+a13*s22;
//                                    k21=a21*s11+a23*s21;
//                                    k22=a21*s12+a23*s22;
//                                    k31=a31*s11+a33*s21;
//                                    k32=a31*s12+a33*s22;
//
//                                    /* a posteriori state update */
//                                    y_est=y_prior+(k11*(data.pos.y-y_prior)+k12*(acell.y-ydd_prior));
//                                    yd_est=yd_prior+(k21*(data.pos.y-y_prior)+k22*(acell.y-ydd_prior));
//                                    ydd_est=ydd_prior+(k31*(data.pos.y-y_prior)+k32*(acell.y-ydd_prior));
//
//                                    /* a posteriori Covariance */
//                                    p11=a11-(k11*(k11*s11+k12*s21)+k12*(k11*s12+k12*s22));
//                                    p12=a12-(k21*(k11*s11+k12*s21)+k22*(k11*s12+k12*s22));
//                                    p13=a13-(k31*(k11*s11+k12*s21)+k32*(k11*s12+k12*s22));
//
//                                    p21=a21-(k11*(k21*s11+k22*s21)+k12*(k21*s12+k22*s22));
//                                    p22=a22-(k21*(k21*s11+k22*s21)+k22*(k21*s12+k22*s22));
//                                    p23=a23-(k31*(k21*s11+k22*s21)+k32*(k21*s12+k22*s22));
//
//                                    p31=a31-(k11*(k31*s11+k32*s21)+k12*(k31*s12+k32*s22));
//                                    p32=a32-(k21*(k31*s11+k32*s21)+k22*(k31*s12+k32*s22));
//                                    p33=a33-(k31*(k31*s11+k32*s21)+k32*(k31*s12+k32*s22));
//
//
//                                    state.y=y_est;
//                                    state_dot.y=yd_est;
//                                    state_ddot.y=ydd_est;
//
//
//                                    /* printf("X=%6.3f\n",state.x); */
//
//                                    }
//             /* just IMU available */
//             if (data.new_data==0){
//                                    /* Kalman Gain, K 3x1 */
//                                    k1=a13/(a33+r22);
//                                    k2=a23/(a33+r22);
//                                    k3=a33/(a33+r22);
//
//                                    /* a posteriori state update */
//                                    y_est=y_prior+k1*(acell.y-ydd_prior);
//                                    yd_est=yd_prior+k2*(acell.y-ydd_prior);
//                                    ydd_est=ydd_prior+k3*(acell.y-ydd_prior);
//
//                                    /* a posteriori Covariance */
//                                    p11=a11-k1*k1/(a33+r22);
//                                    p12=a12-k1*k2/(a33+r22);
//                                    p13=a13-k1*k3/(a33+r22);
//
//                                    p21=a21-k2*k1/(a33+r22);
//                                    p22=a22-k2*k2/(a33+r22);
//                                    p23=a23-k2*k3/(a33+r22);
//
//                                    p31=a31-k3*k1/(a33+r22);
//                                    p32=a32-k3*k2/(a33+r22);
//                                    p33=a33-k3*k3/(a33+r22);
//                                    state.y=y_est;
//                                    state_dot.y=yd_est;
//                                    state_ddot.y=ydd_est;
//
//
//                                    }
//
//	pos_est->y=state.y;
//
//    data.new_data=0;
//
//}

/*@}*/
