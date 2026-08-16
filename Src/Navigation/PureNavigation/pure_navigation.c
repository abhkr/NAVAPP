/*
 * navigation.c
 *
 *  Created on: 12-Aug-2026
 *      Author: ngarm-ins
 */

#include "pure_navigation.h"

#include <math.h>
#include <stddef.h>

#include "dcm.h"
#include "quaternion.h"
#include "vector3.h"
#include "matrix.h"
#include "wgs84.h"
#include "transform.h"

#define PURE_NAVIGATION_GRAVITY_M_S2       (9.80665)
#define PURE_NAVIGATION_MIN_DT_S           (1.0e-9)
#define PURE_NAVIGATION_HALF               (0.5)

static void PureNavigation_RemoveBias(const Vector3_t *measurement,
		const Vector3_t *bias, double dt_s, Vector3_t *bias_delta);

static bool PureNavigation_UpdateAttitude(PureNavigationState_t *navigation,
		const Vector3_t *delta_angle_rad);

static bool PureNavigation_UpdateVelocity(PureNavigationState_t *navigation,
		const Vector3_t *delta_velocity_m_s, double dt_s);

static bool PureNavigation_UpdatePosition(PureNavigationState_t *navigation,
		double dt_s);

static Vector3_t PureNavigation_GetGravityNed(void);

void PureNavigation_Init(PureNavigationState_t *navigation) {
	if (navigation != NULL) {
		Wgs84_Init(&navigation->earth_model);

		navigation->solution.position.latitude_rad = 0.0;
		navigation->solution.position.longitude_rad = 0.0;
		navigation->solution.position.altitude_m = 0.0;

		Vector3_Zero(&navigation->solution.velocity_ned_m_s);

		Quaternion_Identity(&navigation->solution.quaternion_ned_to_body);

		Matrix3_Identity(&navigation->solution.dcm_ned_to_body);

		navigation->solution.time_s = 0.0;
		navigation->solution.valid = false;

		Vector3_Zero(&navigation->sensor_bias.gyro_bias_rad_s);

		Vector3_Zero(&navigation->sensor_bias.accel_bias_m_s2);

		navigation->sensor_bias.valid = false;

		navigation->initialized = true;
	}
}

void PureNavigation_SetInitialState(PureNavigationState_t *navigation,
		const NavigationSolution_t *solution) {
	if ((navigation != NULL) && (solution != NULL)) {
		navigation->solution = *solution;
		navigation->initialized = true;
	}
}

void PureNavigation_SetBias(PureNavigationState_t *navigation,
		const Vector3_t *gyro_bias, const Vector3_t *accel_bias) {
	if ((navigation != NULL) && (gyro_bias != NULL) && (accel_bias != NULL)) {
		navigation->sensor_bias.gyro_bias_rad_s = *gyro_bias;

		navigation->sensor_bias.accel_bias_m_s2 = *accel_bias;

		navigation->sensor_bias.valid = true;
	}
}

bool PureNavigation_Update(PureNavigationState_t *navigation,
		const NavigationImuDelta_t *imu_delta) {
	Vector3_t corrected_delta_angle;
	Vector3_t corrected_delta_velocity;
	double dt_s;

	bool status;

	status = false;

	if ((navigation != NULL) && (imu_delta != NULL)
			&& (navigation->initialized == true)
			&& (imu_delta->valid == true)) {
		dt_s = NAVIGATION_IMU_INTERVAL_S;

		PureNavigation_RemoveBias(&imu_delta->delta_angle_rad,
				&navigation->sensor_bias.gyro_bias_rad_s, dt_s,
				&corrected_delta_angle);

		PureNavigation_RemoveBias(&imu_delta->delta_velocity_m_s,
				&navigation->sensor_bias.accel_bias_m_s2, dt_s,
				&corrected_delta_velocity);

		if (PureNavigation_UpdateAttitude(navigation,
				&corrected_delta_angle) == true) {
			if (PureNavigation_UpdateVelocity(navigation,
					&corrected_delta_velocity, dt_s) == true) {
				if (PureNavigation_UpdatePosition(navigation, dt_s) == true) {
					navigation->solution.time_s = imu_delta->time_s;

					navigation->solution.valid = true;

					status = true;
				}
			}
		}
	}

	return status;
}

bool PureNavigation_GetSolution(const PureNavigationState_t *navigation,
		NavigationSolution_t *solution) {
	bool status;

	status = false;

	if ((navigation != NULL) && (solution != NULL)
			&& (navigation->solution.valid == true)) {
		*solution = navigation->solution;
		status = true;
	}

	return status;
}

static void PureNavigation_RemoveBias(const Vector3_t *measurement,
		const Vector3_t *bias, double dt_s, Vector3_t *bias_delta) {

	Vector3_Scale(bias, dt_s, bias_delta);

	Vector3_Subtract(measurement, bias_delta, bias_delta);
}

static bool PureNavigation_UpdateAttitude(PureNavigationState_t *navigation,
		const Vector3_t *delta_angle_rad) {
	Quaternion_t delta_quaternion;
	Quaternion_t updated_quaternion;

	bool status;

	status = false;

	if ((navigation != NULL) && (delta_angle_rad != NULL)) {
		/*
		 * The quaternion represents NED -> Body.
		 *
		 * Body-frame incremental rotation is
		 * therefore post-multiplied.
		 */
		delta_quaternion = Quaternion_FromRotationVector(delta_angle_rad);

		updated_quaternion = Quaternion_Multiply(
				&navigation->solution.quaternion_ned_to_body,
				&delta_quaternion);

		if (Quaternion_Normalize(&updated_quaternion) == true) {
			navigation->solution.quaternion_ned_to_body = updated_quaternion;

			Dcm_FromQuaternion(&navigation->solution.quaternion_ned_to_body,
					&navigation->solution.dcm_ned_to_body);

			Transform_DcmToEuler(&navigation->solution.dcm_ned_to_body,
					&navigation->solution.attitude_rad);

			status = true;
		}
	}

	return status;
}

static bool PureNavigation_UpdateVelocity(PureNavigationState_t *navigation,
		const Vector3_t *delta_velocity_m_s, double dt_s) {
	Vector3_t delta_velocity_ned_m_s;
	Vector3_t gravity_ned_m_s2;
	Vector3_t gravity_delta_m_s;

	if ((navigation == NULL) || (delta_velocity_m_s == NULL)
			|| (dt_s <= PURE_NAVIGATION_MIN_DT_S)) {
		return false;
	}

	/*
	 * Specific force is measured in body frame.
	 *
	 * C_NED_to_Body maps NED -> Body.
	 * Therefore Body -> NED is C transpose.
	 */

	Matrix3_Transpose(&navigation->solution.dcm_ned_to_body,
			&navigation->solution.dcm_body_to_ned);

	Dcm_MultiplyVector(&navigation->solution.dcm_body_to_ned,
			delta_velocity_m_s, &delta_velocity_ned_m_s);

	gravity_ned_m_s2 = PureNavigation_GetGravityNed();

	Vector3_Scale(&gravity_ned_m_s2, dt_s, &gravity_delta_m_s);

	/*
	 * NED gravity is positive down.
	 *
	 * Specific force and gravity are combined here.
	 */
	Vector3_Add(&delta_velocity_ned_m_s, &gravity_delta_m_s,
			&delta_velocity_ned_m_s);

	Vector3_Add(&navigation->solution.velocity_ned_m_s, &delta_velocity_ned_m_s,
			&navigation->solution.velocity_ned_m_s);

	return true;
}

static bool PureNavigation_UpdatePosition(PureNavigationState_t *navigation,
		double dt_s) {
	double latitude_rad;
	double longitude_rad;
	double altitude_m;

	double north_velocity_m_s;
	double east_velocity_m_s;
	double down_velocity_m_s;

	/*
	 * This first implementation uses a local flat-earth
	 * approximation.
	 *
	 * WGS-84 radii and Earth rotation should replace this
	 * section for the production mechanization.
	 */
	if ((navigation == NULL) || (dt_s <= PURE_NAVIGATION_MIN_DT_S)) {
		return false;
	}

	latitude_rad = navigation->solution.position.latitude_rad;

	longitude_rad = navigation->solution.position.longitude_rad;

	altitude_m = navigation->solution.position.altitude_m;

	north_velocity_m_s = navigation->solution.velocity_ned_m_s.x;

	east_velocity_m_s = navigation->solution.velocity_ned_m_s.y;

	down_velocity_m_s = navigation->solution.velocity_ned_m_s.z;

	/*
	 * Local-earth approximation.
	 *
	 * These constants are intentionally kept here only until
	 * the WGS-84 Earth model is implemented in Math/Navigation.
	 */
	latitude_rad += (north_velocity_m_s / (6378137.0 + altitude_m)) * dt_s;

	if (fabs(cos(latitude_rad)) > 1.0e-12) {
		longitude_rad += (east_velocity_m_s
				/ ((6378137.0 + altitude_m) * cos(latitude_rad))) * dt_s;
	}

	altitude_m -= down_velocity_m_s * dt_s;

	navigation->solution.position.latitude_rad = latitude_rad;

	navigation->solution.position.longitude_rad = longitude_rad;

	navigation->solution.position.altitude_m = altitude_m;

	return true;
}

static Vector3_t PureNavigation_GetGravityNed(void) {
	Vector3_t gravity;

	gravity.x = 0.0;
	gravity.y = 0.0;
	gravity.z = PURE_NAVIGATION_GRAVITY_M_S2;

	return gravity;
}
