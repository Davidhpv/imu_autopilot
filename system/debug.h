/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

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
* @file Debug buffering and output
*   @author Lorenz Meier
*
*/

#ifndef DEBUG_H_
#define DEBUG_H_

#include <inttypes.h>
#include <comm.h>
#include <mavlink.h>
#include "mav_vect.h"

#define DEBUG_COUNT 16
#define DEBUG_MAX_LEN MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN

#define DEBUG_VECT_NAME_MAX_LEN MAVLINK_MSG_DEBUG_VECT_FIELD_NAME_LEN

void debug_message_init(void);

/** @brief Buffer one debug message */
uint8_t debug_message_buffer(const char* string);

/** @brief Buffer one debug message with a integer variable in it */
uint8_t debug_message_buffer_sprintf(const char* string, const uint32_t num);

/** @brief Immediately send one debug message */
void debug_message_send_immediate(mavlink_channel_t chan, const char* text);

/** @brief Send one of the buffered messages */
void debug_message_send_one(void);

void debug_vect(const char* string,const float_vect3 vect);


#endif /* DEBUG_H_ */
