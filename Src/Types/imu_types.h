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
 * @brief Raw IMU measurement.
 *
 * Gyroscope:
 *     rad/s
 *
 * Accelerometer:
 *     m/s^2
 */
typedef struct {
	Vector3_t gyro_rad_s;

	Vector3_t accel_m_s2;

	uint64_t timestamp_us;

	bool valid;

} ImuMeasurement_t;

/**
 * @brief IMU sample buffer.
 *
 * Four samples are collected at 2.5 ms intervals
 * to form one 10 ms navigation update.
 */
#define IMU_SAMPLES_PER_NAV_UPDATE    (4U)

typedef struct {
	ImuMeasurement_t samples[IMU_SAMPLES_PER_NAV_UPDATE];

	uint32_t sample_count;

} ImuSampleBuffer_t;

/**
 * @brief IMU sensor configuration.
 */
typedef struct {
	double sample_period_s;

	double gyro_scale;

	double accel_scale;

} ImuConfiguration_t;

#endif /* TYPES_IMU_TYPES_H_ */
