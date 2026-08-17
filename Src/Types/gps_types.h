/*
 * gps_types.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef TYPES_GPS_TYPES_H_
#define TYPES_GPS_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief GPS position measurement.
 *
 * Latitude  : radians
 * Longitude : radians
 * Altitude  : metres
 */
typedef struct {
	double latitude_rad;

	double longitude_rad;

	double altitude_m;

	bool valid;

} GpsPosition_t;

/**
 * @brief GPS velocity measurement.
 *
 * Velocity is expressed in NED frame.
 *
 * Units: m/s
 */
typedef struct {
	double north_m_s;

	double east_m_s;

	double down_m_s;

	bool valid;

} GpsVelocity_t;

/**
 * @brief Complete GPS measurement.
 */
typedef struct {
	GpsPosition_t position;

	GpsVelocity_t velocity;

	uint8_t satellites;

	double horizontal_accuracy_m;

	double vertical_accuracy_m;

	uint64_t timestamp_us;

	bool valid;

} GpsMeasurement_t;

/**
 * @brief GPS receiver configuration.
 */
typedef struct {
	double update_period_s;

} GpsConfiguration_t;

#endif /* TYPES_GPS_TYPES_H_ */
