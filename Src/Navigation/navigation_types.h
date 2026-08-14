/*
 * navigation_types.h
 *
 *  Created on: 12-Aug-2026
 *      Author: abhimanyu
 */

#ifndef NAVIGATION_TYPES_H_
#define NAVIGATION_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "math_types.h"
#include "wgs84.h"

#define NAVIGATION_IMU_INTERVAL_S       (0.010)
#define NAVIGATION_GPS_INTERVAL_S       (1.000)
#define NAVIGATION_IMU_SAMPLES_PER_UPDATE (4U)

typedef struct {
	Vector3_t delta_angle_rad;
	Vector3_t delta_velocity_m_s;

	double time_s;

	bool valid;
} NavigationImuDelta_t;

typedef struct {
	GeodeticPosition_t position;

	Vector3_t velocity_ned_m_s;

	Quaternion_t quaternion_ned_to_body;
	Matrix3_t dcm_ned_to_body;
	Matrix3_t dcm_body_to_ned;

	EulerAngles_t attitude_rad;

	double time_s;

	bool valid;
} NavigationSolution_t;

typedef struct {
	Vector3_t gyro_bias_rad_s;
	Vector3_t accel_bias_m_s2;

	bool valid;
} NavigationSensorBias_t;

typedef struct {
	NavigationSolution_t solution;

	NavigationSensorBias_t sensor_bias;

	Wgs84Ellipsoid_t earth_model;

	bool initialized;
} PureNavigationState_t;

typedef struct {
	GeodeticPosition_t position;
	Vector3_t velocity_ned_m_s;

	double time_s;

	bool position_valid;
	bool velocity_valid;
} NavigationGpsMeasurement_t;

typedef struct {
	Vector3_t position_error_m;
	Vector3_t velocity_error_m_s;

	Vector3_t attitude_error_rad;

	Vector3_t gyro_bias_error_rad_s;
	Vector3_t accel_bias_error_m_s2;
} NavigationErrorState_t;

typedef struct {
	NavigationErrorState_t error_state;

	bool initialized;
	bool gps_update_valid;
} HybridNavigationState_t;

#endif /* NAVIGATION_TYPES_H_ */
