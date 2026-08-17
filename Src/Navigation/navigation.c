/*
 * navigation.c
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#include "navigation.h"

#include <stddef.h>

#include "attitude.h"
#include "gravity.h"
#include "position.h"
#include "velocity.h"

static void Navigation_ClearImuBuffer(Navigation_t *navigation);

static bool Navigation_ProcessImuSamples(Navigation_t *navigation,
		NavigationImuIncrement_t *imu);

static void Navigation_CalculateImuAverage(const Navigation_t *navigation,
		Vector3_t *gyro_average, Vector3_t *accel_average);

static void Navigation_ResetImuSampleCount(Navigation_t *navigation);

void Navigation_Init(Navigation_t *navigation) {
	if (navigation != NULL) {
		Position_Init(&navigation->position);

		Velocity_Init(&navigation->velocity);

		Attitude_Init(&navigation->attitude);

		Gravity_Init(&navigation->gravity);

		navigation->earth_rate.earth_rate_ned_rad_s.x = 0.0;

		navigation->earth_rate.earth_rate_ned_rad_s.y = 0.0;

		navigation->earth_rate.earth_rate_ned_rad_s.z = 0.0;

		navigation->transport_rate.transport_rate_ned_rad_s.x = 0.0;

		navigation->transport_rate.transport_rate_ned_rad_s.y = 0.0;

		navigation->transport_rate.transport_rate_ned_rad_s.z = 0.0;

		Navigation_ClearImuBuffer(navigation);

		Gravity_Update(&navigation->gravity, &navigation->position);

		navigation->initialized = true;
	}
}

bool Navigation_ImuIsr(Navigation_t *navigation, const ImuMeasurement_t *sample) {
	NavigationImuIncrement_t imu;
	bool update_ready;

	update_ready = false;

	if ((navigation != NULL) && (sample != NULL)
			&& (navigation->initialized == true)) {
		if (sample->valid == true) {
			if (navigation->imu_sample_count <
			NAVIGATION_IMU_SAMPLE_COUNT) {
				navigation->imu_samples[navigation->imu_sample_count] = *sample;

				navigation->imu_sample_count++;
			}

			if (navigation->imu_sample_count ==
			NAVIGATION_IMU_SAMPLE_COUNT) {
				update_ready = Navigation_ProcessImuSamples(navigation, &imu);

				if (update_ready == true) {
					Navigation_Update(navigation, &imu);
				}

				Navigation_ResetImuSampleCount(navigation);
			}
		}
	}

	return update_ready;
}

void Navigation_Update(Navigation_t *navigation,
		const NavigationImuIncrement_t *imu) {
	if ((navigation != NULL) && (imu != NULL) && (imu->valid == true)
			&& (navigation->initialized == true)) {
		/*
		 * Attitude propagation.
		 */
		Attitude_Propagate(&navigation->attitude, imu);

		/*
		 * Gravity update.
		 */
		Gravity_Update(&navigation->gravity, &navigation->position);

		/*
		 * Velocity propagation.
		 */
		Velocity_Propagate(&navigation->velocity, &navigation->attitude,
				&navigation->gravity, imu);

		/*
		 * Position propagation.
		 */
		Position_Propagate(&navigation->position, &navigation->velocity,
				imu->dt_s);
	}
}

void Navigation_GetSolution(const Navigation_t *navigation,
		NavigationSolution_t *solution) {
	if ((navigation != NULL) && (solution != NULL)
			&& (navigation->initialized == true)) {
		solution->position = navigation->position;

		solution->velocity = navigation->velocity;

		solution->attitude = navigation->attitude;

		solution->gravity = navigation->gravity;

		solution->earth_rate = navigation->earth_rate;

		solution->transport_rate = navigation->transport_rate;

		solution->valid = true;
	}
}

bool Navigation_IsInitialized(const Navigation_t *navigation) {
	bool initialized;

	initialized = false;

	if (navigation != NULL) {
		initialized = navigation->initialized;
	}

	return initialized;
}

static bool Navigation_ProcessImuSamples(Navigation_t *navigation,
		NavigationImuIncrement_t *imu) {
	Vector3_t gyro_average;
	Vector3_t accel_average;

	bool valid;

	valid = false;

	if ((navigation != NULL) && (imu != NULL)) {
		Navigation_CalculateImuAverage(navigation, &gyro_average,
				&accel_average);

		imu->delta_angle_rad.x = gyro_average.x *
		NAVIGATION_UPDATE_PERIOD_S;

		imu->delta_angle_rad.y = gyro_average.y *
		NAVIGATION_UPDATE_PERIOD_S;

		imu->delta_angle_rad.z = gyro_average.z *
		NAVIGATION_UPDATE_PERIOD_S;

		imu->delta_velocity_m_s.x = accel_average.x *
		NAVIGATION_UPDATE_PERIOD_S;

		imu->delta_velocity_m_s.y = accel_average.y *
		NAVIGATION_UPDATE_PERIOD_S;

		imu->delta_velocity_m_s.z = accel_average.z *
		NAVIGATION_UPDATE_PERIOD_S;

		imu->dt_s =
		NAVIGATION_UPDATE_PERIOD_S;

		imu->valid = true;

		valid = true;
	}

	return valid;
}

static void Navigation_CalculateImuAverage(const Navigation_t *navigation,
		Vector3_t *gyro_average, Vector3_t *accel_average) {
	uint32_t index;

	gyro_average->x = 0.0;
	gyro_average->y = 0.0;
	gyro_average->z = 0.0;

	accel_average->x = 0.0;
	accel_average->y = 0.0;
	accel_average->z = 0.0;

	for (index = 0U; index < NAVIGATION_IMU_SAMPLE_COUNT; index++) {
		gyro_average->x += navigation->imu_samples[index].gyro_rad_delt.x;

		gyro_average->y += navigation->imu_samples[index].gyro_rad_delt.y;

		gyro_average->z += navigation->imu_samples[index].gyro_rad_delt.z;

		accel_average->x += navigation->imu_samples[index].accel_m_s_delt.x;

		accel_average->y += navigation->imu_samples[index].accel_m_s_delt.y;

		accel_average->z += navigation->imu_samples[index].accel_m_s_delt.z;
	}

	gyro_average->x /= (double) NAVIGATION_IMU_SAMPLE_COUNT;

	gyro_average->y /= (double) NAVIGATION_IMU_SAMPLE_COUNT;

	gyro_average->z /= (double) NAVIGATION_IMU_SAMPLE_COUNT;

	accel_average->x /= (double) NAVIGATION_IMU_SAMPLE_COUNT;

	accel_average->y /= (double) NAVIGATION_IMU_SAMPLE_COUNT;

	accel_average->z /= (double) NAVIGATION_IMU_SAMPLE_COUNT;
}

static void Navigation_ClearImuBuffer(Navigation_t *navigation) {
	uint32_t index;

	navigation->imu_sample_count = 0U;

	for (index = 0U; index < NAVIGATION_IMU_SAMPLE_COUNT; index++) {
		navigation->imu_samples[index].valid = false;
	}
}

static void Navigation_ResetImuSampleCount(Navigation_t *navigation) {
	navigation->imu_sample_count = 0U;
}
