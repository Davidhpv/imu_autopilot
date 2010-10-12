/*
 * comm_temp.c
 *
 *  Created on: Apr 30, 2009
 *      Author: mavteam
 */
#include "downlink.h"
#include "comm_temp.h"
#include "protocol.h"
#include "comm.h"

void comm_send_string(char *string){
	while(*string){
		comm_send_ch(COMM_1, *string++);
	}
}

void comm_send_int(int i){
	char buffer[11];
	comm_int_to_string(i,buffer);
	comm_send_string(buffer);
}

void comm_int_to_string(int i, char* buffer){
	if(i<0){
		buffer[0]='-';
		i=-i;
	}
	else{
		buffer[0]=' ';
	}
	buffer[9]=i%10+48;
	buffer[8]=(i/10)%10+48;
	buffer[7]=(i/100)%10+48;
	buffer[6]=(i/1000)%10+48;
	buffer[5]=(i/10000)%10+48;
	buffer[4]=(i/100000)%10+48;
	buffer[3]=(i/1000000)%10+48;
	buffer[2]=(i/10000000)%10+48;
	buffer[1]=(i/100000000)%10+48;
	buffer[10]=0;
}
