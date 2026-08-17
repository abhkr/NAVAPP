/*
 * navigation.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef NAVIGATION_NAVIGATION_H_
#define NAVIGATION_NAVIGATION_H_

#include <stdbool.h>
#include <stdint.h>

#include "imu_types.h"
#include "navigation_types.h"

#define NAVIGATION_IMU_SAMPLE_COUNT       (4U)

#define NAVIGATION_IMU_SAMPLE_PERIOD_S    (0.0025)

#define NAVIGATION_UPDATE_PERIOD_S        (0.0100)

/**
 * @brief Navigation system state.
 */
typedef struct {
	NavigationPosition_t position;

	NavigationVelocity_t velocity;

	NavigationAttitude_t attitude;

	NavigationGravity_t gravity;

	NavigationEarthRate_t earth_rate;

	NavigationTransportRate_t transport_rate;

	/*
	 * IMU sample accumulation.
	 */
	ImuMeasurement_t imu_samples[NAVIGATION_IMU_SAMPLE_COUNT];

	uint32_t imu_sample_count;

	bool initialized;

} Navigation_t;

/**
 * @brief Initialize navigation system.
 *
 * @param navigation Pointer to navigation state.
 */
void Navigation_Init(Navigation_t *navigation);

/**
 * @brief Process one IMU sample.
 *
 * This function is called at 400 Hz.
 *
 * Four samples are accumulated and the navigation
 * solution is propagated at 100 Hz.
 *
 * @param navigation Pointer to navigation state.
 * @param sample Pointer to IMU measurement.
 *
 * @return true when a navigation update is completed.
 * @return false otherwise.
 */
bool Navigation_ImuIsr(Navigation_t *navigation, const ImuMeasurement_t *sample);

/**
 * @brief Perform one navigation propagation update.
 *
 * @param navigation Pointer to navigation state.
 * @param imu Pointer to processed IMU increment.
 */
void Navigation_Update(Navigation_t *navigation,
		const NavigationImuIncrement_t *imu);

/**
 * @brief Get the current navigation solution.
 *
 * @param navigation Pointer to navigation state.
 * @param solution Pointer to output solution.
 */
void Navigation_GetSolution(const Navigation_t *navigation,
		NavigationSolution_t *solution);

/**
 * @brief Check navigation initialization state.
 *
 * @param navigation Pointer to navigation state.
 *
 * @return true if initialized.
 * @return false otherwise.
 */
bool Navigation_IsInitialized(const Navigation_t *navigation);

#endif /* NAVIGATION_NAVIGATION_H_ */
