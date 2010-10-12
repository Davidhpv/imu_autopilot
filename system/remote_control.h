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
 *   @brief Definition of the class remote_control.
 *
 *   @author
 *
 */
 




#ifndef REMOTE_CONTROL_H_
#define REMOTE_CONTROL_H_

///defines the triggerpoints for special command signals (e.g. start or stop Motors)
#define PPM_LOW_TRIG  1200
#define PPM_HIGH_TRIG  1800

#define PPM_OFFSET 			1000
#define PPM_SCALE_FACTOR 	0.001f
#define PPM_CENTRE 			 1500

void remote_control(void);



#endif /* REMOTE_CONTROL_H_ */


