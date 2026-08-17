/*
 * imu_calibration.h
 *
 *  Created on: 12-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef IMU_CALIBRATION_H_
#define IMU_CALIBRATION_H_

#include "math_types.h"
#include "imu_types.h"

/**
 * @brief IMU calibration parameters.
 */
typedef struct {
	Vector3_t gyro_bias;
	Vector3_t accel_bias;

	Matrix3_t gyro_matrix;
	Matrix3_t accel_matrix;
} ImuCalibration_t;

/**
 * @brief Initialize calibration parameters.
 */
void ImuCalibration_Init(ImuCalibration_t *calibration);

/**
 * @brief Apply gyroscope calibration.
 */
void ImuCalibration_ApplyGyro(const ImuCalibration_t *calibration,
		const Vector3_t *raw_gyro, Vector3_t *corrected);

/**
 * @brief Apply accelerometer calibration.
 */
void ImuCalibration_ApplyAccel(const ImuCalibration_t *calibration,
		const Vector3_t *raw_accel, Vector3_t *corrected);

/**
 * @brief Set gyroscope bias.
 */
SensorStatus_t ImuCalibration_SetGyroBias(ImuCalibration_t *calibration,
		const Vector3_t *bias);

/**
 * @brief Set accelerometer bias.
 */
SensorStatus_t ImuCalibration_SetAccelBias(ImuCalibration_t *calibration,
		const Vector3_t *bias);

#endif /* IMU_CALIBRATION_H_ */
