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
#include "wgs84.h"

/* ==========================================================================
 * Navigation constants
 * ========================================================================== */

#define NAVIGATION_IMU_SAMPLE_COUNT    (4U)

/* ==========================================================================
 * Basic vector types
 * ========================================================================== */

/* ==========================================================================
 * Mission Data Load Parameters
 * ========================================================================== */

typedef struct {
	GeodeticPosition_t position;
	NedVelocity_t velocity;
	EulerAngles_t attitude;
	float64_t leveling_time;
	float64_t navigation_time;
	bool updated;

} NavigationMdl_t;

/* ==========================================================================
 * Navigation solution
 * ========================================================================== */

typedef struct {
	GeodeticPosition_t position;
	NedVelocity_t velocity;
	EulerAngles_t attitude;
	Quaternion_t quaternion;
} NavigationSolution_t;

/* ==========================================================================
 * Navigation state
 * ========================================================================== */

typedef struct {
	NavigationSolution_t pure_solution;
	/*
	 * Earth model quantities.
	 */
	Wgs84AngularRates_t rates;
	Wgs84Radii_t radius;

	float64_t gravity_mps2;
	Matrix3_t dcm_ned_to_body;
	Matrix3_t dcm_body_to_ned;

	ImuMeasurement_t imu_compensated;
	ImuMeasurement_t imu_samples[NAVIGATION_IMU_SAMPLE_COUNT];

	uint32_t imu_sample_count;
	uint32_t rcnt;

} Navigation_t;

#endif /* TYPES_NAVIGATION_TYPES_H_ */
