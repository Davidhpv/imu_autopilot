#include "transformation.h"
#include "mav_vect.h"
#include <math.h>

void turn_xy_plane(const float_vect3* vector, const float yaw,
		float_vect3* result)
{
	//turn clockwise
	result->x = cos(yaw) * vector->x + sin(yaw) * vector->y;
	result->y = -sin(yaw) * vector->x + cos(yaw) * vector->y;
	result->z = vector->z; //leave direction normal to xy-plane untouched

}

void navi2body_xy_plane(const float_vect3* vector, const float yaw,
		float_vect3* result)
{
	turn_xy_plane(vector, yaw, result);
	//	result->x = cos(yaw) * vector->x + sin(yaw) * vector->y;
	//	result->y = -sin(yaw) * vector->x + cos(yaw) * vector->y;
	//	result->z = vector->z; //leave direction normal to xy-plane untouched
}
void body2navi_xy_plane(const float_vect3* vector, const float yaw,
		float_vect3* result)
{
	turn_xy_plane(vector, -yaw, result);
	//	result->x = cos(yaw) * vector->x + -sin(yaw) * vector->y;
	//	result->y = sin(yaw) * vector->x + cos(yaw) * vector->y;
	//	result->z = vector->z; //leave direction normal to xy-plane untouched
}

void navi2body(const float_vect3* vector, const float_vect3* angles,
		float_vect3* result)
{
	//TODO implement this
}
//Laurens
void body2navi(const float_vect3* vector, const float_vect3* angles,
		float_vect3* result)
{
	result->x = cos(angles->y) * cos(angles->z) * vector->x

	+ (-cos(angles->x) * sin(angles->z) + sin(angles->x) * sin(angles->y)
			* cos(angles->z)) * vector->y

	+ (cos(angles->x) * sin(angles->y) * cos(angles->z) + sin(angles->x) * sin(
			angles->z)) * vector->z;


	result->y = (cos(angles->y) * sin(angles->z)) * vector->x

	+ (cos(angles->x) * cos(angles->z) + sin(angles->x) * sin(angles->y) * sin(
			angles->z)) * vector->y

	+ (cos(angles->x) * sin(angles->y) * sin(angles->z) - sin(angles->x) * cos(
			angles->z)) * vector->z;


	result->z = (-sin(angles->y)) * vector->x

	+ (sin(angles->x) * cos(angles->y)) * vector->y

	+ cos(angles->x) * cos(angles->y) * vector->z;
}
//
////TOBI WORKING FOR YAW=0 roll is somtimes wrong for big angles
//void body2navi(const float_vect3* vector, const float_vect3* angles,
//		float_vect3* result)
//{
//	result->x = cos(angles->y) * cos(angles->z) * vector->x
//
//	+ (-cos(angles->x) * sin(angles->z) - sin(angles->x) * sin(angles->y)
//			* cos(angles->z)) * vector->y
//
//	+ (cos(angles->x) * sin(angles->y) * cos(angles->z) + sin(angles->x) * sin(
//			angles->z)) * vector->z;
//
//
//	result->y = (cos(angles->y) * sin(angles->z)) * vector->x
//
//	+ (cos(angles->x) * cos(angles->z) + sin(angles->x) * sin(angles->y) * sin(
//			angles->z)) * vector->y
//
//	+ (cos(angles->x) * sin(angles->y) * sin(angles->z) - sin(angles->x) * cos(
//			angles->z)) * vector->z;
//
//
//	result->z = (-sin(angles->y)) * vector->x
//
//	+ (sin(angles->x) * cos(angles->y)) * vector->y
//
//	+ cos(angles->x) * cos(angles->y) * vector->z;
//}

//calculates the yaw from the z-gyro


////TOBI2
//void body2world(const float_vect3* vector, const float_vect3* angles, float_vect3*  result)
//{
//	result->x = cos(angles->y) * cos(angles->z) * vector->x
//
//			+ cos(angles->y) * sin(
//			angles->z) * vector->y
//
//			- sin(angles->y) * vector->z;
//
//	result->y = (sin(angles->x) * sin(angles->y) * cos(angles->z) - cos(angles->x)
//			* sin(angles->z)) * vector->x
//
//			+ (cos(angles->x) * cos(angles->z)
//			+ sin(angles->x) * sin(angles->y) * sin(angles->z)) * vector->y
//
//			+ sin(angles->x) * cos(angles->y) * vector->z;
//
//	result->z = (cos(angles->x) * sin(angles->y) * cos(angles->z) + sin(angles->x)
//			* sin(angles->z)) * vector->x
//
//			+ (cos(angles->x) * sin(angles->y) * sin(
//			angles->z) - sin(angles->x) * cos(angles->z)) * vector->y
//
//			+ cos(angles->x) * cos(angles->y) * vector->z;
//}


// AMIR new
// Transforms from Body Frame to Navigation Frame(NED->North, East,Down, Origin:Take Off Point)
//void body2Nav(const float_vect3* vector, const float_vect3* angles, float_vect3*  result)
//{
//	result->x = cos(angles->y) * cos(angles->z) * vector->x
//				+(cos(angles->z)*sin(angles->x) * sin(angles->y)-cos(angles->x)*sin(angles->z)) * vector->y
//				+(sin(angles->x)*sin(angles->z)+cos(angles->x)*cos(angles->z)*sin(angles->y)) * vector->z;
//
//	result->y = cos(angles->y))* sin(angles->z) * vector->x
//			+ (cos(angles->x) * cos(angles->z)+ sin(angles->x) * sin(angles->y) * sin(angles->z)) * vector->y
//			+ (cos(angles->x) * sin(angles->y) * sin(angles->z)-cos(angles->z)*sin(angles->x))* vector->z;
//
//	result->z = - sin(angles->y) * vector->x
//				+ cos(angles->y) * sin(angles->x) * vector->y
//				+ cos(angles->x) * cos(angles->y) * vector->z;
//}


//void body2world(const float_vect3* vector, const float_vect3* angles, float_vect3*  result)
//{
//	result->x = cos(angles->y) * cos(angles->z) * vector->x
//
//			+ cos(angles->y) * sin(
//			angles->z) * vector->y
//
//			- sin(angles->y) * vector->z;
//
//	result->y = (-sin(angles->x) * sin(angles->y) * cos(angles->z) - cos(angles->x)
//			* sin(angles->z)) * vector->x
//
//			+ (-sin(angles->x) * sin(angles->y)
//			* sin(angles->z) + cos(angles->x) * cos(angles->z)) * vector->y
//
//			- sin(angles->x) * cos(angles->y) * vector->z;
//
//	result->z = (cos(angles->x) * sin(angles->y) * cos(angles->z) - sin(angles->x)
//			* sin(angles->z)) * vector->x
//
//			+ (cos(angles->x) * sin(angles->y) * sin(
//			angles->z) + sin(angles->x) * cos(angles->z)) * vector->y
//
//			+ cos(angles->x) * cos(angles->y) * vector->z;
//}

// AMIR
//void body2world(const float_vect3* vector, const float_vect3* angles, float_vect3*  result)
//{
//	result->x = cos(angles->y) * cos(angles->z) * vector->x + cos(angles->y) * sin(
//			angles->z) * vector->y - sin(angles->y) * vector->z;
//
//	result->y = (-sin(angles->x) * sin(angles->y) * cos(angles->z) - cos(angles->x)
//			* sin(angles->z)) * vector->x
//			+ (-sin(angles->x) * sin(angles->y)
//			* sin(angles->z) + cos(angles->x) * cos(angles->z)) * vector->y
//			- sin(angles->x) * cos(angles->y) * vector->z;
//
//	result->z = (cos(angles->x) * sin(angles->y) * cos(angles->z) - sin(angles->x)
//			* sin(angles->z)) * vector->x + (cos(angles->x) * sin(angles->y) * sin(
//			angles->z) + sin(angles->x) * cos(angles->z)) * vector->y + cos(
//			angles->x) * cos(angles->y) * vector->z;
//}
