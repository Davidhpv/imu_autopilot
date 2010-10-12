/*=====================================================================
 
PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>
 
(c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>
 
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
 *   @brief Definition all features that are possible.
 *
 *   @author Laurens Mackay
 *
 */


#ifndef FEATURES_H_
#define FEATURES_H_

/* LED */
//#define FEATURE_LED_DISABLED					1
//#define FEATURE_LED_ENABLED						2

//#define FEATURE_EEPROM						0

/* Motor controllers */
#define FEATURE_MOTORCONTROLLER_DISABLED		1
#define FEATURE_MOTORCONTROLLER_MIKROKOPTER_PWM	2
#define FEATURE_MOTORCONTROLLER_PIXHAWK_RPM		3

#define FEATURE_SENSOR_PRESSURE_INTERUPT_DISABLED	1
#define FEATURE_SENSOR_PRESSURE_INTERUPT_ENABLED	2



#endif /* FEATURES_H_ */


