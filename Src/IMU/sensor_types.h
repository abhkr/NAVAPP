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
	float64_t gyro_x;
	float64_t gyro_y;
	float64_t gyro_z;

	float64_t accel_x;
	float64_t accel_y;
	float64_t accel_z;

	uint32_t timestamp_us;

	bool valid;
} ImuRawData_t;

#endif /* SENSOR_TYPES_H_ */
