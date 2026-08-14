/*
 * hybrid_navigation.c
 *
 *  Created on: 12-Aug-2026
 *      Author: ngarm-ins
 */

#include "hybrid_navigation.h"

#include <stddef.h>

#include "vector3.h"

static void HybridNavigation_ComputePositionResidual(
		HybridNavigationState_t *hybrid,
		const PureNavigationState_t *pure_navigation,
		const NavigationGpsMeasurement_t *gps);

static void HybridNavigation_ComputeVelocityResidual(
		HybridNavigationState_t *hybrid,
		const PureNavigationState_t *pure_navigation,
		const NavigationGpsMeasurement_t *gps);

static void HybridNavigation_ApplyCorrection(HybridNavigationState_t *hybrid,
		PureNavigationState_t *pure_navigation);

void HybridNavigation_Init(HybridNavigationState_t *hybrid) {
	if (hybrid != NULL) {
		Vector3_Zero(&hybrid->error_state.position_error_m);

		Vector3_Zero(&hybrid->error_state.velocity_error_m_s);

		Vector3_Zero(&hybrid->error_state.attitude_error_rad);

		Vector3_Zero(&hybrid->error_state.gyro_bias_error_rad_s);

		Vector3_Zero(&hybrid->error_state.accel_bias_error_m_s2);

		hybrid->initialized = true;
		hybrid->gps_update_valid = false;
	}
}

bool HybridNavigation_Update(HybridNavigationState_t *hybrid,
		PureNavigationState_t *pure_navigation,
		const NavigationGpsMeasurement_t *gps) {
	bool status;

	status = false;

	if ((hybrid != NULL) && (pure_navigation != NULL) && (gps != NULL)
			&& (hybrid->initialized == true)
			&& (pure_navigation->initialized == true)
			&& (pure_navigation->solution.valid == true)) {
		if (gps->position_valid == true) {
			HybridNavigation_ComputePositionResidual(hybrid, pure_navigation,
					gps);
		}

		if (gps->velocity_valid == true) {
			HybridNavigation_ComputeVelocityResidual(hybrid, pure_navigation,
					gps);
		}

		/*
		 * This is the integration point for the
		 * 15-state error-state EKF.
		 *
		 * Production flow:
		 *
		 *   residual
		 *       ↓
		 *   measurement matrix H
		 *       ↓
		 *   innovation covariance
		 *       ↓
		 *   Kalman gain
		 *       ↓
		 *   error-state estimate
		 *       ↓
		 *   feedback correction
		 */
		HybridNavigation_ApplyCorrection(hybrid, pure_navigation);

		hybrid->gps_update_valid = true;

		status = true;
	}

	return status;
}

bool HybridNavigation_GetErrorState(const HybridNavigationState_t *hybrid,
		NavigationErrorState_t *error_state) {
	bool status;

	status = false;

	if ((hybrid != NULL) && (error_state != NULL)
			&& (hybrid->gps_update_valid == true)) {
		*error_state = hybrid->error_state;

		status = true;
	}

	return status;
}

static void HybridNavigation_ComputePositionResidual(
		HybridNavigationState_t *hybrid,
		const PureNavigationState_t *pure_navigation,
		const NavigationGpsMeasurement_t *gps) {
	hybrid->error_state.position_error_m.x = (gps->position.latitude_rad
			- pure_navigation->solution.position.latitude_rad) * 6378137.0;

	hybrid->error_state.position_error_m.y = (gps->position.longitude_rad
			- pure_navigation->solution.position.longitude_rad) * 6378137.0;

	hybrid->error_state.position_error_m.z = gps->position.altitude_m
			- pure_navigation->solution.position.altitude_m;
}

static void HybridNavigation_ComputeVelocityResidual(
		HybridNavigationState_t *hybrid,
		const PureNavigationState_t *pure_navigation,
		const NavigationGpsMeasurement_t *gps) {
	Vector3_Subtract(&gps->velocity_ned_m_s,
			&pure_navigation->solution.velocity_ned_m_s,
			&hybrid->error_state.velocity_error_m_s);
}

static void HybridNavigation_ApplyCorrection(HybridNavigationState_t *hybrid,
		PureNavigationState_t *pure_navigation) {
	/*
	 * Do not directly force GPS position/velocity into the
	 * INS solution.
	 *
	 * This function intentionally remains empty until the
	 * error-state EKF is implemented.
	 *
	 * The EKF feedback will eventually correct:
	 *
	 *   position
	 *   velocity
	 *   attitude
	 *   gyro bias
	 *   accelerometer bias
	 */

	(void) hybrid;
	(void) pure_navigation;
}
