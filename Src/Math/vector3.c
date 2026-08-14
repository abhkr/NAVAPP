/*
 * vector3.c
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */
#include <stdlib.h>
#include <math.h>

#include "vector3.h"
#include "math_constants.h"

void Vector3_Create(double x, double y, double z, Vector3_t *result) {

	result->x = x;
	result->y = y;
	result->z = z;
}

void Vector3_Zero(Vector3_t *result) {

	result->x = 0.0;
	result->y = 0.0;
	result->z = 0.0;
}

void Vector3_Add(const Vector3_t *lhs, const Vector3_t *rhs, Vector3_t *result) {

	result->x = lhs->x + rhs->x;
	result->y = lhs->y + rhs->y;
	result->z = lhs->z + rhs->z;
}

void Vector3_Subtract(const Vector3_t *lhs, const Vector3_t *rhs,
		Vector3_t *result) {

	result->x = lhs->x - rhs->x;
	result->y = lhs->y - rhs->y;
	result->z = lhs->z - rhs->z;
}

void Vector3_Scale(const Vector3_t *vector, double scale, Vector3_t *result) {

	result->x = vector->x * scale;
	result->y = vector->y * scale;
	result->z = vector->z * scale;
}

double Vector3_Dot(const Vector3_t *lhs, const Vector3_t *rhs) {
	double result;

	result = (lhs->x * rhs->x) + (lhs->y * rhs->y) + (lhs->z * rhs->z);

	return result;
}

void Vector3_Cross(const Vector3_t *lhs, const Vector3_t *rhs,
		Vector3_t *result) {

	result->x = (lhs->y * rhs->z) - (lhs->z * rhs->y);
	result->y = (lhs->z * rhs->x) - (lhs->x * rhs->z);
	result->z = (lhs->x * rhs->y) - (lhs->y * rhs->x);
}

double Vector3_Norm(const Vector3_t *vector) {
	double norm_squared;
	double norm;

	norm_squared = (vector->x * vector->x) + (vector->y * vector->y)
			+ (vector->z * vector->z);

	norm = sqrt(norm_squared);

	return norm;
}

MathStatus_t Vector3_Normalize(Vector3_t *vector) {
	double norm;

	if (vector == NULL) {
		return MATH_STATUS_NULL_POINTER;
	}

	norm = Vector3_Norm(vector);

	if (norm <= MATH_EPSILON) {
		return MATH_STATUS_INVALID_NORM;
	}

	vector->x /= norm;
	vector->y /= norm;
	vector->z /= norm;

	return MATH_STATUS_OK;
}
