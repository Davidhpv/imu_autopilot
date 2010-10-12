/*======================================================================

 PIXHAWK mavlib - The Micro Air Vehicle Platform Library
 Please see our website at <http://pixhawk.ethz.ch>


 Original Authors:

 Contributing Authors (in alphabetical order):
 Joseph Kruijen, jkruijen@student.ethz.ch




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
#ifndef _FAST_ATAN2_H_
#define _FAST_ATAN2_H_

#include "inttypes.h"
#include "math.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define PI  3.1415926535897932384626433832795029

/** @brief Fast @c abs(x)
 ** @param x argument.
 ** @return @c abs(x)
 **/
static inline float fast_abs(float x)
{
	return (x >= 0) ? x : -x;
}

/** @brief Fast @c atan2
 ** @param x argument.
 ** @param y argument.
 ** @return Approximation of @c atan2(x).
 **/
static inline float fast_atan2(float y, float x)
{

	/*
	 The function f(r)=atan((1-r)/(1+r)) for r in [-1,1] is easier to
	 approximate than atan(z) for z in [0,inf]. To approximate f(r) to
	 the third degree we may solve the system

	 f(+1) = c0 + c1 + c2 + c3 = atan(0) = 0
	 f(-1) = c0 - c1 + c2 - c3 = atan(inf) = pi/2
	 f(0)  = c0                = atan(1) = pi/4

	 which constrains the polynomial to go through the end points and
	 the middle point.

	 We still miss a constrain, which might be simply a constarint on
	 the derivative in 0. Instead we minimize the Linf error in the
	 range [0,1] by searching for an optimal value of the free
	 parameter. This turns out to correspond to the solution

	 c0=pi/4, c1=-0.9675, c2=0, c3=0.1821

	 which has maxerr = 0.0061 rad = 0.35 grad.
	 */

	float angle, r;
	const float c3 = 0.1821;
	const float c1 = 0.9675;
	float abs_y = fast_abs(y) + (float) (1e-10);

	if (x >= 0)
	{
		r = (x - abs_y) / (x + abs_y);
		angle = (float) (PI / 4.0);
	}
	else
	{
		r = (x + abs_y) / (abs_y - x);
		angle = (float) (3 * PI / 4.0);
	}
	angle += (c3 * r * r - c1) * r;
	return (y < 0) ? -angle : angle;
}

#ifdef __cplusplus
}
#endif

#endif /* _FAST_ATAN2_H_ */
