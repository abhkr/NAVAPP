/*
 * quaternion.c
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#include "quaternion.h"

#include <math.h>
#include <stdlib.h>

#include "math_constants.h"

MathStatus_t Quaternion_Identity(Quaternion_t *result) {

	if (result == NULL) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->w = 1.0;
	result->x = 0.0;
	result->y = 0.0;
	result->z = 0.0;

	return MATH_STATUS_OK;
}

MathStatus_t Quaternion_Create(float64_t w, float64_t x, float64_t y,
		float64_t z, Quaternion_t *result) {

	if (result == NULL) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->w = w;
	result->x = x;
	result->y = y;
	result->z = z;

	return MATH_STATUS_OK;
}

MathStatus_t Quaternion_Multiply(const Quaternion_t *lhs,
		const Quaternion_t *rhs, Quaternion_t *result) {

	if ((lhs == NULL) || (rhs == NULL) || (result == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->w = (lhs->w * rhs->w) - (lhs->x * rhs->x) - (lhs->y * rhs->y)
			- (lhs->z * rhs->z);

	result->x = (lhs->w * rhs->x) + (lhs->x * rhs->w) + (lhs->y * rhs->z)
			- (lhs->z * rhs->y);

	result->y = (lhs->w * rhs->y) - (lhs->x * rhs->z) + (lhs->y * rhs->w)
			+ (lhs->z * rhs->x);

	result->z = (lhs->w * rhs->z) + (lhs->x * rhs->y) - (lhs->y * rhs->x)
			+ (lhs->z * rhs->w);

	return MATH_STATUS_OK;
}

double Quaternion_Norm(const Quaternion_t *quaternion) {
	double norm_squared;
	double norm;

	if (quaternion == NULL) {
		norm_squared = 0.0;
	} else {

		norm_squared = (quaternion->w * quaternion->w)
				+ (quaternion->x * quaternion->x)
				+ (quaternion->y * quaternion->y)
				+ (quaternion->z * quaternion->z);
	}

	norm = sqrt(norm_squared);

	return norm;
}

MathStatus_t Quaternion_Normalize(Quaternion_t *quaternion) {
	double norm;

	if (quaternion == NULL) {
		return MATH_STATUS_NULL_POINTER;
	}

	norm = Quaternion_Norm(quaternion);

	if (norm <= MATH_EPSILON) {
		return MATH_STATUS_INVALID_NORM;
	}

	quaternion->w /= norm;
	quaternion->x /= norm;
	quaternion->y /= norm;
	quaternion->z /= norm;

	return MATH_STATUS_OK;
}

MathStatus_t Quaternion_Conjugate(const Quaternion_t *quaternion,
		Quaternion_t *result) {

	if ((quaternion == NULL) || (result == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}
	result->w = quaternion->w;
	result->x = -quaternion->x;
	result->y = -quaternion->y;
	result->z = -quaternion->z;

	return MATH_STATUS_OK;
}

MathStatus_t Quaternion_Inverse(const Quaternion_t *quaternion,
		Quaternion_t *result) {
	double norm_squared;

	if ((quaternion == NULL) || (result == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}

	norm_squared = (quaternion->w * quaternion->w)
			+ (quaternion->x * quaternion->x) + (quaternion->y * quaternion->y)
			+ (quaternion->z * quaternion->z);

	result->w = quaternion->w / norm_squared;
	result->x = -quaternion->x / norm_squared;
	result->y = -quaternion->y / norm_squared;
	result->z = -quaternion->z / norm_squared;

	return MATH_STATUS_OK;
}

MathStatus_t Quaternion_FromRotationVector(const Vector3_t *rotation_vector_rad,
		Quaternion_t *result) {
	double angle;
	double half_angle;
	double scale;

	if ((rotation_vector_rad == NULL) || (result == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}

	angle = sqrt(
			(rotation_vector_rad->x * rotation_vector_rad->x)
					+ (rotation_vector_rad->y * rotation_vector_rad->y)
					+ (rotation_vector_rad->z * rotation_vector_rad->z));

	if (angle <= MATH_EPSILON) {
		Quaternion_Identity(result);
	} else {
		half_angle = 0.5 * angle;
		scale = sin(half_angle) / angle;

		result->w = cos(half_angle);
		result->x = rotation_vector_rad->x * scale;
		result->y = rotation_vector_rad->y * scale;
		result->z = rotation_vector_rad->z * scale;
	}

	return MATH_STATUS_OK;
}
