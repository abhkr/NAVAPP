/*
 * imu.c
 *
 *  Created on: 12-Aug-2026
 *      Author: ngarm-ins
 */

#include "imu.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "vector3.h"
#include "wgs84.h"
#include "transform.h"
#include "matrix.h"
#include "math_constants.h"

void Imu_Init(Imu_t *imu) {
	if (imu != NULL) {
		ImuCalibration_Init(&imu->calibration);

		Vector3_Zero(&imu->measurement.gyro_rad_delt);
		Vector3_Zero(&imu->measurement.accel_m_s_delt);
		imu->measurement.valid = false;

		imu->initialized = true;
	}
}

SensorStatus_t Imu_SetCalibration(Imu_t *imu,
		const ImuCalibration_t *calibration) {
	if ((imu == NULL) || (calibration == NULL)) {
		return SENSOR_STATUS_NULL_POINTER;
	}

	imu->calibration = *calibration;

	return SENSOR_STATUS_OK;
}

SensorStatus_t Imu_Acquire(ImuMeasurement_t *measurement) {

	if (measurement == NULL) {
		return SENSOR_STATUS_NULL_POINTER;
	}

//	measurement->gyro_rad_s.x = 0.0;
//	measurement->gyro_rad_s.y = 0.0;
//	measurement->gyro_rad_s.z = 0.0;
//
//	measurement->accel_m_s2.x = 0.0;
//	measurement->accel_m_s2.y = 0.0;
//	measurement->accel_m_s2.z = 0.0;

	return SENSOR_STATUS_OK;
}

SensorStatus_t Imu_AcquireStatic(ImuMeasurement_t *measurement,
		const float64_t lattitude, const float64_t del_t, const float64_t g_val,
		const Matrix3_t *dcm_ned2body) {

	Vector3_t gyro_rad_delt;
	Vector3_t acc_m_s_delt;

	if (measurement == NULL) {
		return SENSOR_STATUS_NULL_POINTER;
	}

	gyro_rad_delt.x = WGS84_EARTH_ROTATION_RAD_S * cos(lattitude);
	gyro_rad_delt.y = 0.0;
	gyro_rad_delt.z = -WGS84_EARTH_ROTATION_RAD_S * sin(lattitude);

	acc_m_s_delt.x = 0.0;
	acc_m_s_delt.y = 0.0;
	acc_m_s_delt.z = -g_val;

	Vector3_Scale(&gyro_rad_delt, del_t, &gyro_rad_delt);

	Vector3_Scale(&acc_m_s_delt, del_t, &acc_m_s_delt);

	Matrix3_MultiplyVector(dcm_ned2body, &gyro_rad_delt,
			&measurement->gyro_rad_delt);

	measurement->gyro_rad_delt.z += ((10.0 * MATH_DEG_TO_RAD) * del_t);

	Matrix3_MultiplyVector(dcm_ned2body, &acc_m_s_delt,
			&measurement->accel_m_s_delt);

	return SENSOR_STATUS_OK;
}

