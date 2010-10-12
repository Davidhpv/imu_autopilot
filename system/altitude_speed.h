/*
 * altitude_speed.h
 *
 *  Created on: Aug 11, 2010
 *      Author: pixhawk
 */

#ifndef ALTITUDE_SPEED_H_
#define ALTITUDE_SPEED_H_

#include "math.h"

#define air_gas_constant 8.31432f
#define air_density_sea_level 1.225f
#define absolute_null_kelvin 273.15f
// Pressure at sea level in pascal
#define pressure_sea_level 101325.0f

static inline float get_air_density(float static_pressure, float temperature_celsius)
{
  return static_pressure/(air_gas_constant * (temperature_celsius + absolute_null_kelvin));
}

/**
 * @brief Calculate indicated airspeed
 *
 * Please note that the indicated airspeed is not the true airspeed because it
 * lacks the air density compensation. Use the calc_true_airspeed functions to get
 * the true airspeed.
 *
 * @parem pressure_front pressure inside the pitot/prandl tube
 * @param pressure_ambient pressure at the side of the tube/airplane
 * @return indicated airspeed in m/s
 */
static inline float calc_indicated_airspeed(float pressure_front, float pressure_ambient)
{
  return sqrt((2.0f*(pressure_front - pressure_ambient))/air_density_sea_level);
}

/**
 * @brief Calculate indicated airspeed
 *
 * Please note that the indicated airspeed is not the true airspeed because it
 * lacks the air density compensation. Use the calc_true_airspeed functions to get
 * the true airspeed.
 *
 * @parem pressure_diff Differential pressure measured by sensor
 * @return indicated airspeed in m/s
 */
static inline float calc_indicated_airspeed_diff(float pressure_diff)
{
  return sqrt((2.0f*(pressure_diff))/air_density_sea_level);
}

/**
 * @brief Calculate true airspeed from indicated airspeed
 *
 * Please note that the true airspeed is NOT the groundspeed, because of the effects of wind
 *
 * @parem speed current indicated airspeed
 * @param pressure_ambient pressure at the side of the tube/airplane
 * @param temperature air temperature in degrees celcius
 * @return true airspeed in m/s
 */
static inline float calc_true_airspeed_from_indicated(float speed, float pressure_ambient, float temperature)
{
  return speed * sqrt(air_density_sea_level/get_air_density(pressure_ambient, temperature));
}

/**
 * @brief Directly calculate true airspeed
 *
 * Please note that the true airspeed is NOT the groundspeed, because of the effects of wind
 *
 * @parem pressure_front pressure inside the pitot/prandl tube
 * @param pressure_ambient pressure at the side of the tube/airplane
 * @param temperature air temperature in degrees celcius
 * @return true airspeed in m/s
 */
static inline float calc_true_airspeed(float pressure_front, float pressure_ambient, float temperature)
{
  return sqrt((2.0f*(pressure_front - pressure_ambient))/get_air_density(pressure_ambient, temperature));
}


static inline float calc_altitude_pressure(float pressure_ambient)
{
	return 44330.0f * ((1-pow((pressure_ambient/pressure_sea_level),(1/5.255f)))); // Meters
}


static inline float get_indicated_airspeed(void)
{
	return calc_indicated_airspeed_diff(global_data.pressure_diff_si);
}

static inline float get_true_airspeed(void)
{
	return calc_true_airspeed(global_data.pressure_diff_si+global_data.pressure_si, global_data.pressure_si, global_data.temperature_si);
}

static inline float get_pressure_altitude(void)
{
	return calc_altitude_pressure(global_data.pressure_si);
}

#endif /* ALTITUDE_SPEED_H_ */
