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
#include "imu_types.h"

/**
 * @brief IMU sensor object.
 */
typedef struct {
	ImuCalibration_t calibration;
	ImuMeasurement_t measurement;

	bool initialized;
} Imu_t;

/**
 * @brief Configure IMU calibration.
 */
SensorStatus_t Imu_SetCalibration(Imu_t *imu,
		const ImuCalibration_t *calibration);

SensorStatus_t Imu_Acquire(ImuMeasurement_t *measurement);

#endif /* IMU_H_ */
