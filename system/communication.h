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
* @file
* @brief Communication reception on both UARTS
*   @author Lorenz Meier
*   @author Laurens MacKay
*/

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "conf/conf.h"
#include <stdbool.h>
#include "comm.h"
#include <mavlink.h>

void execute_action(uint8_t action);

/** @addtogroup COMM */
//@{
/** @name Communication functions
*  abstraction layer for comm */
//@{
void handle_mavlink_message(mavlink_channel_t chan, mavlink_message_t* msg);

/**
* @brief Send low-priority messages at a maximum rate of xx Hertz
*
* This function sends messages at a lower rate to not exceed the wireless
* bandwidth. It sends one message each time it is called until the buffer is empty.
* Call this function with xx Hertz to increase/decrease the bandwidth.
*/
void communication_queued_send(void);



void communication_init(void);

/**
* @brief Receive communication packets and handle them
*
* This function decodes packets on the protocol level and also handles
* their value by calling the appropriate functions.
*/
void communication_receive(void);

//@}}

#endif /* COMMUNICATION_H_ */
