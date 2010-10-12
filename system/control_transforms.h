// FIXME DELETE?

/*
 * control_transforms.h
 *
 *  Created on: Sep 11, 2009
 *      Author: pixhawk
 */

#ifndef CONTROL_TRANSFORMS_H_
#define CONTROL_TRANSFORMS_H_

#include "fast_atan2.h"

#if 0
/**
 * @param yaw = phi
 * @param pitch = theta
 * @param roll = psi
 */
static inline void calculate_body_to_world_matrix(float yaw, float pitch, float roll, float *R)
{
	/*
	 * Using euler ordering,
	 * 	{C,S}1 = {cos,sin}(roll)
	 * 	{C,S}2 = {cos,sin}(pitch)
	 * 	{C,S}3 = {cos,sin}(yaw)
	 */
	float ct = cos(pitch);
	float st = sin(pitch);
	float cph = cos(yaw);
	float sph = sin(yaw);
	float cps = cos(roll);
	float sps = sin(roll);

	R[0] = ct * cph; 		R[1] = sps * st * cph - cps * sph;		R[2] = cps * st * cph + sps * sph;
	R[3] = ct * sph;		R[4] = sps * st * sph + cps * cph;		R[5] = cps * st * sph - sps * cph;
	R[6] = -st;				R[7] = sps * ct;						R[8] = cps * ct;
}

static inline void transpose_R(float *R)
{
	float RTemp[9];

	for(int i=0; i<9; i++) RTemp[i] = R[i];

	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			R[j*3+i] = RTemp[i*3+j];
}

/**
 * This function computes euler angles from the rotation matrix
 *
 * @param R Rotation matrix, 3x3
 * @param yaw = phi
 * @param pitch = theta
 * @param roll = psi
 */
static inline void calculate_euler_angles_from_R(float *R, float *yaw_p, float *pitch_p, float *roll_p)
{
	float phi, theta, psi;  /* euler angles */
	float delta;
	float ct;

    /* note: there are two hypotheses; I just choose one */
	if (fab(R[6]) != 1) {
	    theta = -asin(R[6]);
	    ct = cos(theta);
	    psi = fast_atan2(R[7] / ct, R[8] / ct);
	    phi = fast_atan2(R[3] / ct, R[0] / ct);
	  }
	  else {
	    phi = 0.0;
	    delta = fast_atan2(R[1], R[2]);
	    if (R[6] == -1) {
	      theta = M_PI/2;
	      psi = phi + delta;
	    }
	    else {
	      theta = -M_PI/2;
	      psi = -phi + delta;
	    }
	  }

	  *yaw_p = phi; *pitch_p = theta; *roll_p = psi;
}
#endif

//static inline void transform_force_to_attitude(float *pitch_desired, float *roll_desired, float *thrust_desired, float fx, float fy, float fz)
//{
//	/* marker_attitude.z = yaw */
//	float c3 = cos(global_data.marker_attitude.z);
//	float s3 = sin(global_data.marker_attitude.z);
//	float weight = 0.46 * 9.8; /* mg, m=460g */
//
//	*pitch_desired = fast_atan2( fz - weight, fx*c3 + fy*s3);
//	*roll_desired = fast_atan2(fz - weight, (fx*s3 - fy*c3) * cos(*pitch_desired));
//	*thrust_desired = (weight - fz) / (cos(*pitch_desired) * cos(*roll_desired));
//}

#endif /* CONTROL_TRANSFORMS_H_ */
