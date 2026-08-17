/*
 * imu.h
 *
 *  Created on: 12-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>

#include "imu_calibration.h"
#include "sensor_types.h"

/**
 * @brief Processed IMU measurement.
 */
typedef struct {
	Vector3_t gyro_rad_s;
	Vector3_t accel_m_s2;

	double timestamp_s;

	bool valid;
} ImuMeasurement_t;

/**
 * @brief IMU sensor object.
 */
typedef struct {
	ImuCalibration_t calibration;
	ImuMeasurement_t measurement;

	bool initialized;
} Imu_t;

/**
 * @brief Initialize IMU processing.
 */
void Imu_Init(Imu_t *imu);

/**
 * @brief Process a raw IMU measurement.
 *
 * This function is called whenever a new IMU sample is available.
 */
SensorStatus_t Imu_Process(Imu_t *imu, const ImuRawData_t *raw_data);

/**
 * @brief Retrieve latest processed measurement.
 */
SensorStatus_t Imu_GetMeasurement(const Imu_t *imu,
		ImuMeasurement_t *measurement);

/**
 * @brief Configure IMU calibration.
 */
SensorStatus_t Imu_SetCalibration(Imu_t *imu,
		const ImuCalibration_t *calibration);

#endif /* IMU_H_ */
