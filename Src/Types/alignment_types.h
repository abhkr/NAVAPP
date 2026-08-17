/*
 * alignment_types.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef TYPES_ALIGNMENT_TYPES_H_
#define TYPES_ALIGNMENT_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "navigation_types.h"
#include "vector3.h"

/**
 * @brief Alignment state.
 */
typedef enum {
	ALIGNMENT_STATE_IDLE = 0,
	ALIGNMENT_STATE_INITIALIZING,
	ALIGNMENT_STATE_RUNNING,
	ALIGNMENT_STATE_COMPLETE,
	ALIGNMENT_STATE_FAILED

} AlignmentState_t;

/**
 * @brief Alignment quality.
 */
typedef enum {
	ALIGNMENT_QUALITY_INVALID = 0,
	ALIGNMENT_QUALITY_LOW,
	ALIGNMENT_QUALITY_MEDIUM,
	ALIGNMENT_QUALITY_HIGH

} AlignmentQuality_t;

/**
 * @brief Alignment attitude result.
 *
 * Angles are radians.
 */
typedef struct {
	double roll_rad;

	double pitch_rad;

	double yaw_rad;

	AlignmentQuality_t quality;

	bool valid;

} AlignmentAttitude_t;

/**
 * @brief Static alignment data.
 */
typedef struct {
	Vector3_t averaged_accel_m_s2;

	Vector3_t averaged_gyro_rad_s;

	double elapsed_time_s;

	uint32_t sample_count;

	AlignmentState_t state;

	AlignmentAttitude_t attitude;

} StaticAlignmentData_t;

/**
 * @brief Transfer alignment data.
 */
typedef struct {
	NavigationPosition_t position;

	NavigationVelocity_t velocity;

	NavigationAttitude_t attitude;

	Vector3_t reference_velocity_m_s;

	Vector3_t reference_angular_rate_rad_s;

	double elapsed_time_s;

	AlignmentState_t state;

	AlignmentAttitude_t alignment_result;

} TransferAlignmentData_t;

#endif /* TYPES_ALIGNMENT_TYPES_H_ */
