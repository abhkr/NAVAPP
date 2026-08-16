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
	double x;
	double y;
	double z;
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
	double m_data[3U][3U];
	struct {
		double m00;
		double m01;
		double m02;

		double m10;
		double m11;
		double m12;

		double m20;
		double m21;
		double m22;
	};
} Matrix3_t;

/*
 * Quaternion:
 *
 * q = [w, x, y, z]
 */
typedef struct {
	double w;
	double x;
	double y;
	double z;
} Quaternion_t;

/*
 * Euler angle representation.
 */
typedef struct {
	double roll_rad;
	double pitch_rad;
	double yaw_rad;
} EulerAngles_t;

/*
 * Position expressed using geodetic coordinates.
 */
typedef struct {
	double latitude_rad;
	double longitude_rad;
	double altitude_m;
} GeodeticPosition_t;

typedef struct {
	double latitude_rate_rad_s;
	double longitude_rate_rad_s;
	double altitude_rate_m_s;
} GeodeticPositionRate_t;

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
