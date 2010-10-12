#include "downlink.h"
#include "communication.h"
#include "imu_max1168.h"
#include "adc.h"
#include "gyros.h"
#include "uart.h"
#include "acceleration.h"
#include <string.h>
#include <stdio.h>

void float_to_char(float number, char* result);
void createPacket(char* packet);

typedef union tfloat TFLOAT;


union tfloat {
    float f;
    int i;
};

void float_to_char(float number, char* result) {

	union tfloat b;
    b.f = number;

	char tmp[8];
    sprintf(tmp, "%.8X", b.i);

	int i;
	for(i=0; i<8; i++){
		result[i]=tmp[7-i];
	}
}

void createPacket(char* packet) {

	// Create packet of length 162 bytes
	//unsigned char packet [162];

	/* Packet content:
	 * 00: debug variable
	 * 01: lower motor speed
	 * 02: upper motor speed
	 * 03: Altitude
	 * 04: gyro X
	 * 05: gyro Y
	 * 06: gyro Z
	 * 07: acc X
	 * 08: acc Y
	 * 09: acc Z
	 * 10: roll
	 * 11: pitch
	 * 12: yaw
	 * 13: z-gyro (legacy code)
	 * 14: battery voltage
	 * 15: received rc throttle
	 * 16: received rc left/right
	 * 17: received rc yaw (z axis rotation)
	 * 18: received rc forward/back
	 * 19: Ultrasound/infrared altitude
	 */

	// Fill packet with data
	packet[0]='<';
	// Set debug to zero
	char tmp[8];
	/*int i;
	for(i=0; i<8; i++){
		tmp[i]='0';
	}*/
	float_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 0], tmp, 8);
	float_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 1], tmp, 8);
	float_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 2], tmp, 8);
	float_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 3], tmp, 8);
	double gyro_yaw = gyros_get_si(GYROS_YAW);
	float_to_char(gyro_yaw,tmp);
	memcpy((char*)&packet[1 + 8 * 4], tmp, 8);
	double gyro_pitch = gyros_get_si(GYROS_PITCH);
	float_to_char(gyro_pitch,tmp);
	memcpy((char*)&packet[1 + 8 * 5], tmp, 8);
	double gyro_roll = gyros_get_si(GYROS_ROLL);
	float_to_char(gyro_roll,tmp);
	memcpy((char*)&packet[1 + 8 * 6], tmp, 8);
	double accel_x = acc_get_si(ACC_X);
	float_to_char(accel_x,tmp);
	memcpy((char*)&packet[1 + 8 * 7], tmp, 8);
	double accel_y = acc_get_si(ACC_Y);
	float_to_char(accel_y,tmp);
	memcpy((char*)&packet[1 + 8 * 8], tmp, 8);
	double accel_z = acc_get_si(ACC_Z);
	float_to_char(accel_z,tmp);
	memcpy((char*)&packet[1 + 8 * 9], tmp, 8);
	float_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 10], tmp, 8);
	//double_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 11], tmp, 8);
	//double_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 12], tmp, 8);
	//double_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 13], tmp, 8);
	//double_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 14], tmp, 8);
	//double_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 15], tmp, 8);
	//double_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 16], tmp, 8);
	//double_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 17], tmp, 8);
	//double_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 18], tmp, 8);
	//double_to_char(0.0f,tmp);
	memcpy((char*)&packet[1 + 8 * 19], tmp, 8);

	packet[161]='>';
}





void sendValues(void){
	char packet[162];

	createPacket(packet);

	if(uart1_check_free_space(162)){
		int i;
		for(i=0;i<162;i++){
			downlink_send(packet[i]);
		}
	}

}

