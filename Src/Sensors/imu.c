/*
 * imu.c
 *
 *  Created on: 12-Aug-2026
 *      Author: ngarm-ins
 */

#include <stdlib.h>

#include "imu.h"
#include "vector3.h"

void Imu_Init(Imu_t *imu) {
	if (imu != NULL) {
		ImuCalibration_Init(&imu->calibration);

		Vector3_Zero(&imu->measurement.gyro_rad_s);
		Vector3_Zero(&imu->measurement.accel_m_s2);
		imu->measurement.timestamp_s = 0.0;
		imu->measurement.valid = false;

		imu->initialized = true;
	}
}

SensorStatus_t Imu_Process(Imu_t *imu, const ImuRawData_t *raw_data) {
	Vector3_t raw_gyro;
	Vector3_t raw_accel;

	if ((imu == NULL) || (raw_data == NULL)) {
		return SENSOR_STATUS_NULL_POINTER;
	}

	if (raw_data->valid == false) {
		return SENSOR_STATUS_INVALID_DATA;
	}

	raw_gyro = Vector3_Create(raw_data->gyro_x, raw_data->gyro_y,
			raw_data->gyro_z);

	raw_accel = Vector3_Create(raw_data->accel_x, raw_data->accel_y,
			raw_data->accel_z);

	ImuCalibration_ApplyGyro(&imu->calibration, &raw_gyro,
			&imu->measurement.gyro_rad_s);

	ImuCalibration_ApplyAccel(&imu->calibration, &raw_accel,
			&imu->measurement.accel_m_s2);

	imu->measurement.timestamp_s = ((double) raw_data->timestamp_us) * 1.0e-6;

	imu->measurement.valid = true;

	return SENSOR_STATUS_OK;
}

SensorStatus_t Imu_GetMeasurement(const Imu_t *imu,
		ImuMeasurement_t *measurement) {
	if ((imu == NULL) || (measurement == NULL)) {
		return SENSOR_STATUS_NULL_POINTER;
	}

	if (imu->measurement.valid == false) {
		return SENSOR_STATUS_NOT_READY;
	}

	*measurement = imu->measurement;

	return SENSOR_STATUS_OK;
}

SensorStatus_t Imu_SetCalibration(Imu_t *imu,
		const ImuCalibration_t *calibration) {
	if ((imu == NULL) || (calibration == NULL)) {
		return SENSOR_STATUS_NULL_POINTER;
	}

	imu->calibration = *calibration;

	return SENSOR_STATUS_OK;
}
