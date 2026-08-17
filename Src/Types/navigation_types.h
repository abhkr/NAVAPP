/*
 * navigation_types.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef TYPES_NAVIGATION_TYPES_H_
#define TYPES_NAVIGATION_TYPES_H_

#include "math_types.h"

/**
 * @brief Navigation position in geodetic coordinates.
 *
 * Latitude  : radians
 * Longitude : radians
 * Altitude  : metres
 */
typedef struct {
	double latitude_rad;
	double longitude_rad;
	double altitude_m;

} NavigationPosition_t;

/**
 * @brief Navigation velocity expressed in NED frame.
 *
 * North : m/s
 * East  : m/s
 * Down  : m/s
 */
typedef struct {
	double north_m_s;
	double east_m_s;
	double down_m_s;

} NavigationVelocity_t;

/**
 * @brief Navigation attitude.
 *
 * Rotation convention:
 *     NED -> Body
 *
 * Quaternion and DCM represent frame rotation.
 */
typedef struct {
	Quaternion_t q_ned_to_body;

	Matrix3_t dcm_ned_to_body;

	double roll_rad;
	double pitch_rad;
	double yaw_rad;

} NavigationAttitude_t;

/**
 * @brief Gravity information.
 *
 * Gravity vector is expressed in NED frame.
 */
typedef struct {
	Vector3_t gravity_ned_m_s2;

	double gravity_m_s2;

} NavigationGravity_t;

/**
 * @brief Earth rotation rate expressed in NED frame.
 *
 * Units: rad/s
 */
typedef struct {
	Vector3_t earth_rate_ned_rad_s;

} NavigationEarthRate_t;

/**
 * @brief Transport rate expressed in NED frame.
 *
 * Units: rad/s
 */
typedef struct {
	Vector3_t transport_rate_ned_rad_s;

} NavigationTransportRate_t;

/**
 * @brief IMU increment over one navigation update period.
 *
 * Delta angle     : radians
 * Delta velocity  : m/s
 * Delta time      : seconds
 */
typedef struct {
	Vector3_t delta_angle_rad;

	Vector3_t delta_velocity_m_s;

	double dt_s;

	bool valid;

} NavigationImuIncrement_t;

/**
 * @brief Complete navigation solution.
 */
typedef struct {
	NavigationPosition_t position;

	NavigationVelocity_t velocity;

	NavigationAttitude_t attitude;

	NavigationGravity_t gravity;

	NavigationEarthRate_t earth_rate;

	NavigationTransportRate_t transport_rate;

	bool valid;

} NavigationSolution_t;

#endif /* TYPES_NAVIGATION_TYPES_H_ */
