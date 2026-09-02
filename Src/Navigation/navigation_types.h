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
 * Navigation state
 * ========================================================================== */

typedef struct {
	GeodeticPosition_t position;
	NedVelocity_t velocity;
	EulerAngles_t attitude;
	Quaternion_t quaternion;

	/*
	 * Mid point values
	 */
	GeodeticPosition_t mid_position;

	/*
	 * Earth model quantities.
	 */
	Wgs84AngularRates_t rates;
	Wgs84Radii_t radius;

	Wgs84Gravity_t gravity;

	Matrix3_t dcm_ned_to_body;
	Matrix3_t dcm_body_to_ned;

	NedVelocity_t mid_velocity;

	ImuMeasurement_t imu_compensated;
	ImuMeasurement_t imu_samples[NAVIGATION_IMU_SAMPLE_COUNT];

	uint32_t imu_sample_count;
	uint32_t rcnt;

} Navigation_t;

#endif /* TYPES_NAVIGATION_TYPES_H_ */
