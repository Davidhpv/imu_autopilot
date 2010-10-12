///*
// * fusion.h
// *
// *  Created on: Aug 24, 2009
// *      Author: mavteam
// */
//
//#ifndef FUSION_H_
//#define FUSION_H_
//
///**
// * @brief Compensate the magnetometer offset with prerecorded calibration values
// */
//static inline void magnet_offset_motor_compensation(void)
//{
//		global_data.magnet_compensated.x = ((global_data.magnet_raw.x-(MAG_SENSOR_OFFSET_X+MAG_MOTOR_OFFSET_X))*MAG_RANGE_FACTOR_X);
//		global_data.magnet_compensated.y = ((global_data.magnet_raw.y-(MAG_SENSOR_OFFSET_Y+MAG_MOTOR_OFFSET_Y))*MAG_RANGE_FACTOR_Y);
//		global_data.magnet_compensated.z = ((global_data.magnet_raw.z-(MAG_SENSOR_OFFSET_Z+MAG_MOTOR_OFFSET_Z))*MAG_RANGE_FACTOR_Z);
//}
//
///**
// * @brief Estimate the position with a complimentary filter
// */
////static inline void position_compl(float marker_confidence, float pos_x, float pos_y, float pos_z, float_vect3 accel_si)
////{
////	// TODO Implement a position estimator using marker position and acceleration.
////	// Implement the si-scaling of the accelerometer for this
////
////	///position determination
////		simple_altitude_moving_average(pos_x, &(global_data.position.x), &(global_data.speed.x));
////		simple_altitude_moving_average(pos_y, &(global_data.position.y), &(global_data.speed.y));
////		simple_altitude_moving_average(pos_z, &(global_data.position.z), &(global_data.speed.z));
////
////}
//#endif /* FUSION_H_ */
