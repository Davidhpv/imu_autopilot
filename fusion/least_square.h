/*
 * least_square.h
 *
 *  Created on: Sep 3, 2009
 *      Author: mav
 */

#ifndef LEAST_SQUARE_H_
#define LEAST_SQUARE_H_

/**
 * @brief
 * @param p is a 2x2 matrix and has to be initialized with big diagonal elements (eg 10^4-10^6) and zero else
 * @param theta return value theta[0]=a, theta[1]=b, theta has to be initialized with 0,0
 */
void least_square(int press_raw, int dist, float *theta, float *p);

#endif /* LEAST_SQUARE_H_ */
