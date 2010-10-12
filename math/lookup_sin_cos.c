#include "lookup_sin_cos.h"


/**
 * @brief lookup_sin @c sin(x)
 * @param angle argument.
 * @return @c sin(x)
 */
float lookup_sin(float angle)
{
	int32_t Int_angle;
	int32_t Int_sin;
	float float_sin;

	//make sure angle is between [-PI,PI]
	while (angle > PI)
		angle -= 2 * PI;
	while (angle < -PI)
		angle += 2 * PI;

	//convert angle to [-PI/2,PI/2] range
	if (angle > PI / 2)
		angle = PI / 2 - (angle - PI / 2);
	else if (angle < -PI / 2)
		angle = -PI / 2 - (angle + PI / 2);

	//convert angle to integer array index
	Int_angle = (int) (angle / (PI / 2) * 6433);

	//lookup sinus in lookup_table_sinus
	if (Int_angle >= 0)
		Int_sin = lookup_table_sin[Int_angle];
	else
		Int_sin = -lookup_table_sin[-Int_angle];

	//transform value back to float
	float_sin = (float) (Int_sin / 16383.0f);
	return float_sin;
	//    return angle;
}

/**
 * @brief lookup_cos @c cos(x)
 * @param angle argument.
 * @return @c cos(x)
 */
float lookup_cos(float angle)
{
	return lookup_sin(angle + PI / 2);
}
