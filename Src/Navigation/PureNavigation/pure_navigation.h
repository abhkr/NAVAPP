/*
 * navigation.h
 *
 *  Created on: 12-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef PURENAVIGATION_PURE_NAVIGATION_H_
#define PURENAVIGATION_PURE_NAVIGATION_H_

#include <stdbool.h>

#include "navigation_types.h"

/**
 * @brief Initialize pure navigation.
 *
 * @param navigation Navigation state.
 */
void PureNavigation_Init(PureNavigationState_t *navigation);

/**
 * @brief Set initial navigation state.
 *
 * @param navigation Navigation state.
 * @param solution Initial navigation solution.
 */
void PureNavigation_SetInitialState(PureNavigationState_t *navigation,
		const NavigationSolution_t *solution);

/**
 * @brief Set navigation sensor biases.
 *
 * @param navigation Navigation state.
 * @param gyro_bias Gyroscope bias.
 * @param accel_bias Accelerometer bias.
 */
void PureNavigation_SetBias(PureNavigationState_t *navigation,
		const Vector3_t *gyro_bias, const Vector3_t *accel_bias);

/**
 * @brief Execute one 100 Hz pure navigation update.
 *
 * The input contains accumulated IMU delta-angle and
 * delta-velocity over the 10 ms navigation interval.
 *
 * @param navigation Navigation state.
 * @param imu_delta IMU delta measurement.
 *
 * @return true if update was successful.
 */
bool PureNavigation_Update(PureNavigationState_t *navigation,
		const NavigationImuDelta_t *imu_delta);

/**
 * @brief Get current navigation solution.
 *
 * @param navigation Navigation state.
 * @param solution Output navigation solution.
 *
 * @return true if solution is valid.
 */
bool PureNavigation_GetSolution(const PureNavigationState_t *navigation,
		NavigationSolution_t *solution);

#endif /* PURENAVIGATION_PURE_NAVIGATION_H_ */
