/*
 * navigation_types.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef TYPES_NAVIGATION_TYPES_H_
#define TYPES_NAVIGATION_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "imu_types.h"
#include "quaternion.h"
#include "dcm.h"

/* ==========================================================================
 * Navigation constants
 * ========================================================================== */

#define NAVIGATION_IMU_SAMPLE_COUNT    (4U)

/* ==========================================================================
 * Basic vector types
 * ========================================================================== */

typedef struct {
	double north_m_s;
	double east_m_s;
	double down_m_s;

} NavigationVelocity_t;

typedef struct {
	double latitude_rad;
	double longitude_rad;
	double altitude_m;

} NavigationPosition_t;

typedef struct {
	EulerAngles_t euler;

} NavigationAttitude_t;

/* ==========================================================================
 * Mission Data Load Parameters
 * ========================================================================== */

typedef struct {
	NavigationPosition_t position;
	NavigationVelocity_t velocity;
	NavigationAttitude_t attitude;
	float64_t leveling_time;
	float64_t navigation_time;
	bool updated;

} NavigationMdl_t;

/* ==========================================================================
 * Navigation solution
 * ========================================================================== */

typedef struct {
	NavigationPosition_t position;
	NavigationVelocity_t velocity;
	NavigationAttitude_t attitude;
	Quaternion_t quaternion;

	/*
	 * Earth model quantities.
	 */
	Vector3_t earth_rate_n_radps;
	Vector3_t transport_rate_n_radps;

	float64_t gravity_mps2;

	float64_t meridian_radius_m;
	float64_t prime_vertical_radius_m;

} NavigationSolution_t;

/* ==========================================================================
 * Navigation state
 * ========================================================================== */

typedef struct {
	NavigationSolution_t pure_solution;
	Matrix3_t dcm_ned_to_body;
	ImuMeasurement_t imu_compensated;
	ImuMeasurement_t imu_samples[NAVIGATION_IMU_SAMPLE_COUNT];
	uint32_t imu_sample_count;
	uint32_t rcnt;

} Navigation_t;

#endif /* TYPES_NAVIGATION_TYPES_H_ */
