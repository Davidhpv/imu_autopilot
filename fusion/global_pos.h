/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
	 Joseph Kruijen <jkruijen@student.ethz.ch>
Contributing Authors (in alphabetical order):
	


(c) 2009 PIXHAWK PROJECT

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


#ifndef _GLOBAL_POS_
#define _GLOBAL_POS_

#include "inttypes.h"
#include "math.h"
#include "global_data.h"

void global_pos(int32_t markerId, float_vect3 positionRaw, float_vect3* position);

#endif //_GLOBAL_POS_


