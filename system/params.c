/*
 * params.c
 *
 *  Created on: 23.06.2010
 *      Author: Laurens Mackay
 */
#include "params.h"
#include "mavlink.h"
#include "debug.h"
#include "eeprom.h"
#include "global_data.h"

//typedef union __generic_32bit
//	{
//		uint8_t b[4];
//		float f;
//		int32_t i;
//		int16_t s;
//	} generic_32bit;

#define EEPROM_PARAM_CHECK_VALUE 1123456789
static uint8_t param_handler_step = 0;
static int32_t param_handler_counter = 0;
uint8_t param_sizecheck = 255;
float_vect3 debug;
void param_write(uint32_t param_id)
{
	generic_32bit param;
	switch (param_id)
	{
	case PARAM_IMU_RESET:
		param.f = 0;
		debug_message_buffer("eeprom param: param IMU_RESET set to 0");
		break;
	case ONBOARD_PARAM_COUNT:
		param.i = EEPROM_PARAM_CHECK_VALUE;// if this value will be changed count of parameters has changed
		debug_message_buffer("eeprom param: security check written");
		break;
	default:
		param.f = global_data.param[param_id];
	}
	eeprom_write(4 * param_id, 4, param.b);

	debug.y = param.f;
}

void param_start_read(uint32_t param_id)
{
	eeprom_start_read(4 * param_id, 4);
}
void param_read_update(uint32_t param_id)
{
	generic_32bit param;
	eeprom_read_data(param.b);
	switch (param_id)
	{
	case PARAM_IMU_RESET:
		debug_message_buffer("eeprom param: param IMU_RESET not overridden");
		break;
	case ONBOARD_PARAM_COUNT:

		if (param.i == EEPROM_PARAM_CHECK_VALUE)// if this value will be changed count of parameters has changed
		{
			debug_message_buffer("eeprom param: security check OK");
			param_sizecheck = 1;
		}
		else
		{
			debug_message_buffer("eeprom param: security check FALSE read abborted");
			debug_message_buffer("YOU SHOULD LOAD PARAMS from your file, set, and write them.");
			param_sizecheck = 0;
		}
		break;
	default:
		global_data.param[param_id] = param.f;
	}

	debug.z = param.f;
}
void param_handler()
{
	//	uint16_t address=1;
	//	uint8_t length=4;
	//	static generic_32bit param;
	//	static uint8_t data = 0;
	//	static generic_32bit rdata;
	//	debug_message_buffer_sprintf("param test: param_handler_counter=%u", param_handler_counter);

	switch (param_handler_step)
	{
	case 0://do nothing
		break;
	case 10://Write one parameter to eeprom
		//		eeprom_write(address, length, param.b);
		//		debug_message_buffer_sprintf("param test: wrote data/1000=%u", (uint32_t)(param.f/1000.f));
		//		param_handler_step++;
		if (param_handler_counter <= ONBOARD_PARAM_COUNT)//as last value we write a security check
		{
			param_write(param_handler_counter++);
		}
		else
		{
			param_handler_counter = 0;
			param_handler_step = 11;

			debug_message_buffer("eeprom param: write all finished");
		}
		break;
	case 11://write finished
		break;
	case 20://start read one parameter from eeprom
		//		eeprom_start_read(address, length);
		param_start_read(param_handler_counter);
		//		debug_message_buffer("param test: start read");
		if(param_sizecheck){
			//continue read
			param_handler_step = 21;
		}else{
			//abbort read
			param_handler_step = 0;
			debug_message_buffer("eeprom param: read aborted (!param_sizecheck)");
		}
		break;
	case 21: //save the parameter read before
		//		eeprom_read_data(rdata.b);
		param_read_update(param_handler_counter--);
		//		debug_message_buffer_sprintf("param test: read data*1000=%u",
		//				(uint32_t) (rdata.f * 1000.f));
		if (param_handler_counter >=0 )// security check should have been read before, but we check it anyway and dont write it to ram
		{
			param_handler_step = 20;
		}
		else
		{
			param_handler_counter = 0;
			debug_message_buffer("eeprom param: read all finished");
			param_handler_step = 23;
		}
		break;
	case 23://read all finished
		break;
	case 30://size check
		break;
	case 31://size check
		break;
	default:
		param_handler_step = 0;
	}

	debug.x = param_handler_counter;
//	debug_vect("param c,w,r", debug);
	//	debug_message_buffer_sprintf("param test: param.i=%i", param.i);
	//	data++;
}
void param_write_all()
{

	debug_message_buffer("eeprom param: starting write all");
	param_handler_counter = 0;
	param_handler_step = 10;
}

void param_read_all()
{

	debug_message_buffer("eeprom param: starting read all");
	param_handler_counter = ONBOARD_PARAM_COUNT;
	param_handler_step = 20;
}
uint8_t param_size_check()
{
	return param_sizecheck; // will be true if not read yet
	//	debug_message_buffer("eeprom param: size check start read");
	//	param_start_read(ONBOARD_PARAM_COUNT);
	//
	//	generic_32bit param;
	//	eeprom_read_data(param.b);
	//
	//	if (param.i == EEPROM_PARAM_CHECK_VALUE)// if this value will be changed count of parameters has changed
	//	{
	//		debug_message_buffer("eeprom param: security check OK");
	//		return 1;
	//	}
	//	else
	//	{
	//		debug_message_buffer("eeprom param: security check FALSE");
	//		return 0;
	//	}

}
