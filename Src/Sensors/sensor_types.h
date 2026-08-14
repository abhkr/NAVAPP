/*
 * sensor_types.h
 *
 *  Created on: 12-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef SENSOR_TYPES_H_
#define SENSOR_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "math_types.h"

/**
 * @brief Generic sensor processing status.
 */
typedef enum {
	SENSOR_STATUS_OK = 0,
	SENSOR_STATUS_NULL_POINTER,
	SENSOR_STATUS_INVALID_DATA,
	SENSOR_STATUS_NOT_READY,
	SENSOR_STATUS_CALIBRATION_ERROR
} SensorStatus_t;

/**
 * @brief Raw IMU measurement received from the driver.
 *
 * Units depend on the IMU driver configuration.
 */
typedef struct {
	double gyro_x;
	double gyro_y;
	double gyro_z;

	double accel_x;
	double accel_y;
	double accel_z;

	uint32_t timestamp_us;

	bool valid;
} ImuRawData_t;

/**
 * @brief Raw GPS measurement received from the driver.
 */
typedef struct {
	double latitude_deg;
	double longitude_deg;
	double altitude_m;

	double velocity_north_m_s;
	double velocity_east_m_s;
	double velocity_down_m_s;

	uint32_t timestamp_ms;

	bool position_valid;
	bool velocity_valid;
} GpsRawData_t;

#endif /* SENSOR_TYPES_H_ */
