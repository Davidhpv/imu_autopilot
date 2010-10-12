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
 
#ifndef _POSITION_KALMAN3_H_
#define _POSITION_KALMAN3_H_

#include "mav_vect.h"
#include "global_data.h"

//typedef struct
//{
//    float x;
//    float y;
//    float z;
//    } float_vect3;
//// struct float_vect3
//{
//    float x;
//    float y; 
//    float z;
//    
//    
//};

//typedef struct {
//        float_vect3 pos;
//        float_vect3 ang;
//
//        float confidence;
//        float time_captured;
//        float comp_end;
//
//        int new_data; // New data available = 1; No new Data = 0
//
//        } vision_t;
        


/** @brief Initialize X,Y,Z Kalman filter */
void position_kalman3_init(float_vect3* pos_est, float_vect3* vel_est);
void x_position_kalman3(vision_t* data, float_vect3* pos_est, float_vect3* vel_est);
void y_position_kalman3(vision_t* data, float_vect3* pos_est, float_vect3* vel_est);
void z_position_kalman3(vision_t* data, float_vect3* pos_est, float_vect3* vel_est);
#endif /* _POSITION_KALMAN3_H_ */
 
/*@}*/


