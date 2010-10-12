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

#include "inttypes.h"
#include "stdlib.h"
#include "global_pos.h"

void global_pos(int32_t markerId, float_vect3 positionRaw, float_vect3* position){

	const float_vect3 MarkerPos[8] = {
										{0,0,0},			//marker 000
										{0,-0.402,0.001},	//marker 001
										{0,-0.8,-0.002},	//marker 002
										{0,-1.2,0.001},		//marker 003
										{0,0,-0.262},		//marker 010
										{0,-0.399,-0.259},	//marker 011
										{0,-0.8,-0.261},	//marker 012
										{0,-1.199,-0.256}	//marker 013
										};

	if(markerId >= 10)markerId-=6;

	(*position).x = positionRaw.x + MarkerPos[markerId].x;
	(*position).y = positionRaw.y + MarkerPos[markerId].y;
	(*position).z = positionRaw.z + MarkerPos[markerId].z;
}



