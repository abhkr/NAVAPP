/*
 * imu_types.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef TYPES_IMU_TYPES_H_
#define TYPES_IMU_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "vector3.h"
#include "imu_types.h"

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
 * @brief Raw IMU measurement.
 *
 * Gyroscope:
 *     rad/s
 *
 * Accelerometer:
 *     m/s^2
 */
typedef struct {
	Vector3_t gyro_rad_delt;

	Vector3_t accel_m_s_delt;

	uint64_t timestamp_us;

	bool valid;

} ImuMeasurement_t;

#endif /* TYPES_IMU_TYPES_H_ */
