/*
 *  
 * Copyright (C) 2008 Marcus Wolschon
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

/** 
 * file gps_nmea.c
 * brief Parser for the NMEA protocol
 *
 * This file is a drop-in replacement for gps_ubx.c
 *
 * TODO: THIS NMEA-PARSER IS NOT WELL TESTED AND INCOMPLETE!!!
 * Status:
 *  Parsing GGA and RMC is complete, GSA and other records are
 *  incomplete.
 */
#ifndef GPS_NMEA_H_
#define GPS_NMEA_H_

#include <inttypes.h>
#include <string.h> 
#include <math.h>
#include <stdbool.h>
#ifdef __linux
// do debug-output if run on the linux-target
#include <stdlib.h>
#endif

#include "uart.h"
#include "gps.h"
//include "gps_ubx.h"
#include "latlong.h"
//WORKAROUNDS Was not compiling
#define FALSE 0
#define TRUE 1

#define PI  3.1415926535897932384626433832795029
#define RadOfDeg(x) (x*PI/180)
//end workarounds

void ubxsend_cfg_rst(uint16_t bbr, uint8_t reset_mode);

////////////////////////////////////////////////////////
//       nmea-parser


/**
 * The buffer, we store one nmea-line in
 * for parsing.
 */

int GpsFixValid(void);

/**
 * parse GPGSA-nmea-messages stored in
 * nmea_msg_buf .
 */
void parse_nmea_GPGSA(void);

/**
 * parse GPRMC NMEA messages stored in
 * nmea_msg_buf. This message contains position
 * and speed information.
 */
void parse_nmea_GPRMC(void);

/**
 * parse GPGGA-nmea-messages stored in
 * nmea_msg_buf .
 */
void parse_nmea_GPGGA(void);

/**
 * parse_ubx() has a complete line.
 * Find out what type of message it is and
 * hand it to the parser for that type.
 */
void parse_gps_msg(void);

/**
 * This is the actual parser.
 * It reads one character at a time
 * setting gps_msg_received to TRUE
 * after a full line.
 */
uint8_t parse_nmea(uint8_t c);

#endif /*GPS_NMEA_H_*/
