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
 *   @author Firstname Lastname <mavteam@student.ethz.ch>
 *
 */
 
/** @addtogroup sensor_fusion */
/*@{*/
 
#ifndef _POSITION_KALMAN_H_
#define _POSITION_KALMAN_H_
 
/** @brief Function that does nothing */
void position_kalman(float xACC,float yACC,float zACC,float rollSpeed,float pitchSpeed,float yawSpeed,float xMag,float yMag,float float zMag,float pressure,float distance,float* roll,float* pitch,float* yaw,float* alt,float* groundDistance);
 
#endif // _POSITION_KALMAN_H_
 
/*@}*/


