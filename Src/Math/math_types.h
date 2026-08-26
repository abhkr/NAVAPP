/*
 * math_types.h
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef MATH_TYPES_H_
#define MATH_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

typedef double float64_t;

/*
 * Mathematical scalar type used throughout the navigation software.
 *
 * double is selected initially because navigation calculations such as
 * latitude/longitude, position propagation and covariance calculations
 * benefit from higher precision.
 */

/*
 * Three-dimensional vector.
 */
typedef struct {
	float64_t x;
	float64_t y;
	float64_t z;
} Vector3_t;

/*
 * 3x3 matrix.
 *
 * Stored in row-major form:
 *
 * | m00 m01 m02 |
 * | m10 m11 m12 |
 * | m20 m21 m22 |
 */
typedef union {
	float64_t m_data[3U][3U];
	struct {
		float64_t m00;
		float64_t m01;
		float64_t m02;

		float64_t m10;
		float64_t m11;
		float64_t m12;

		float64_t m20;
		float64_t m21;
		float64_t m22;
	};
} Matrix3_t;

/*
 * Quaternion:
 *
 * q = [w, x, y, z]
 */
typedef struct {
	float64_t w;
	float64_t x;
	float64_t y;
	float64_t z;
} Quaternion_t;

/*
 * Euler angle representation.
 */
typedef struct {
	float64_t roll_rad;
	float64_t pitch_rad;
	float64_t yaw_rad;
} EulerAngles_t;

/*
 * Position expressed using geodetic coordinates.
 */
typedef struct {
	float64_t latitude_rad;
	float64_t longitude_rad;
	float64_t altitude_m;
} GeodeticPosition_t;

typedef struct {
	float64_t latitude_rate_rad_s;
	float64_t longitude_rate_rad_s;
	float64_t altitude_rate_m_s;
} GeodeticPositionRate_t;

typedef struct {
	float64_t north_m_s;
	float64_t east_m_s;
	float64_t down_m_s;

} NedVelocity_t;

/*
 * Mathematical operation status.
 */
typedef enum {
	MATH_STATUS_OK = 0,
	MATH_STATUS_NULL_POINTER,
	MATH_STATUS_INVALID_ARGUMENT,
	MATH_STATUS_SINGULAR_MATRIX,
	MATH_STATUS_INVALID_NORM
} MathStatus_t;

#endif /* MATH_TYPES_H_ */
