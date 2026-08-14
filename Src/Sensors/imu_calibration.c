/*
 * imu_calibration.c
 *
 *  Created on: 12-Aug-2026
 *      Author: ngarm-ins
 */

#include "imu_calibration.h"

#include "matrix.h"
#include "vector3.h"

#include <stdlib.h>

void ImuCalibration_Init(ImuCalibration_t *calibration) {
	if (calibration != NULL) {
		Vector3_Zero(&calibration->gyro_bias);
		Vector3_Zero(&calibration->accel_bias);

		Matrix3_Identity(&calibration->gyro_matrix);
		Matrix3_Identity(&calibration->accel_matrix);
	}
}

void ImuCalibration_ApplyGyro(const ImuCalibration_t *calibration,
		const Vector3_t *raw_gyro, Vector3_t *corrected) {

	Vector3_Subtract(raw_gyro, &calibration->gyro_bias, corrected);

	Matrix3_MultiplyVector(&calibration->gyro_matrix, corrected, corrected);
}

void ImuCalibration_ApplyAccel(const ImuCalibration_t *calibration,
		const Vector3_t *raw_accel, Vector3_t *corrected) {

	Vector3_Subtract(raw_accel, &calibration->accel_bias, corrected);

	Matrix3_MultiplyVector(&calibration->accel_matrix, corrected, corrected);
}

SensorStatus_t ImuCalibration_SetGyroBias(ImuCalibration_t *calibration,
		const Vector3_t *bias) {
	if ((calibration == NULL) || (bias == NULL)) {
		return SENSOR_STATUS_NULL_POINTER;
	}

	calibration->gyro_bias = *bias;

	return SENSOR_STATUS_OK;
}

SensorStatus_t ImuCalibration_SetAccelBias(ImuCalibration_t *calibration,
		const Vector3_t *bias) {
	if ((calibration == NULL) || (bias == NULL)) {
		return SENSOR_STATUS_NULL_POINTER;
	}

	calibration->accel_bias = *bias;

	return SENSOR_STATUS_OK;
}
