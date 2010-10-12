/*
 * $Id: gps.h 5124 2010-07-22 15:07:54Z dewagter $
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
 * @file
 *  @brief Device independent GPS code
 *
*/


#ifndef GPS_H
#define GPS_H

#include "stdbool.h"
//#include "gps_ubx.h"
#include "gps_nmea.h"
#include "global_data.h"

#define GPS_NB_CHANNELS 16

extern uint8_t gps_mode; /* Receiver status */
extern uint16_t gps_week;    /* weeks */
extern uint32_t gps_itow;    /* ms */
extern int32_t  gps_alt;    /* cm       */
extern uint16_t gps_gspeed;  /* cm/s     */
extern int16_t  gps_climb;  /* m/s     */
extern int16_t  gps_course; /* decideg     */
extern int32_t gps_utm_east, gps_utm_north; /** cm */
extern uint8_t gps_utm_zone;
extern int32_t gps_lat, gps_lon; /* 1e7 deg */
extern uint16_t gps_PDOP;
extern uint32_t gps_Pacc, gps_Sacc;
extern uint8_t gps_numSV;
extern uint8_t gps_configuring;

extern uint16_t last_gps_msg_t; /** cputime of the last gps message */

static inline uint8_t gps_fix_valid(void)
{
	if (gps_mode == 3)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t gps_device_mode;

static inline void gps_init(void)
{
	gps_device_mode = global_data.param[PARAM_GPS_MODE] - (((int32_t)global_data.param[PARAM_GPS_MODE]) / 10);
}

static inline void gps_configure(void)
{

}

/** @brief Read out the contents of a GPS message into the variables */
extern void gps_msg_parse(void);

static inline uint8_t gps_parse(uint8_t c)
{
	//return parse_ubx(c);
	return parse_nmea(c);
}


extern volatile uint8_t gps_msg_received;
extern bool gps_pos_available;
//extern uint8_t gps_nb_ovrn;

/** Number of scanned satellites */
extern uint8_t gps_nb_channels;

/** Space Vehicle Information */

// See http://www.alphamicro.net/resources/u-blox/u-blox-6/u-blox6_ProtocolSpecification_Public_(GPS-SW-09017).pdf
struct svinfo
{
  uint8_t svid;     ///< Space vehicle / PRN ID
  uint8_t flags;    ///< Flags
  uint8_t qi;		///< U-blox quality indicator codes: 0: This channel is idle 1: Channel is searching 2: Signal aquired 3: Signal detected but unusable 4: Code Lock on Signal 5, 6, 7: Code and Carrier locked
  uint8_t cno;		///< U-blox signal to noise ratio
  int8_t elev;		///< Elevation in degrees
  int16_t azim;     ///< Degrees azimuth
};

/**
 * @brief Convert a U-Blox qi value to satellite used
 * @param qi U-Blox quality indicator
 * @return 1 if the satellite is used, 0 else
 */
static inline uint8_t gps_satellite_used(uint8_t qi)
{
	return (qi > 4) ? 1 : 0;
}

extern struct svinfo gps_svinfos[GPS_NB_CHANNELS];

//#include "uart.h"

//#define __GpsLink(dev, _x) dev##_x
//#define _GpsLink(dev, _x)  __GpsLink(dev, _x)
//#define GpsLink(_x) _GpsLink(GPS_LINK, _x)
//
//#define GpsBuffer() GpsLink(ChAvailable())
//#define ReadGpsBuffer() { while (GpsLink(ChAvailable())&&!gps_msg_received) parse_ubx(GpsLink(Getch())); }
//#define GpsUartSend1(c) GpsLink(Transmit(c))
//#define GpsUartInitParam(_a,_b,_c) GpsLink(InitParam(_a,_b,_c))
//#define GpsUartRunning GpsLink(TxRunning)
//#define GpsUartSendMessage GpsLink(SendMessage)


#endif /* GPS_H */
