#include <stdlib.h>
#include <math.h>

#include "vector3.h"
#include "math_constants.h"

MathStatus_t Vector3_Create(double x, double y, double z, Vector3_t *result) {

	if (result == NULL) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->x = x;
	result->y = y;
	result->z = z;

	return MATH_STATUS_OK;
}

MathStatus_t Vector3_Assign(const Vector3_t *lhs, Vector3_t *result) {

	if (lhs == NULL || result == NULL) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->x = lhs->x;
	result->y = lhs->y;
	result->z = lhs->z;

	return MATH_STATUS_OK;
}

MathStatus_t Vector3_Zero(Vector3_t *result) {

	if (result == NULL) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->x = 0.0;
	result->y = 0.0;
	result->z = 0.0;

	return MATH_STATUS_OK;
}

MathStatus_t Vector3_Add(const Vector3_t *lhs, const Vector3_t *rhs,
		Vector3_t *result) {

	if ((lhs == NULL) || (rhs == NULL) || result == NULL) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->x = lhs->x + rhs->x;
	result->y = lhs->y + rhs->y;
	result->z = lhs->z + rhs->z;

	return MATH_STATUS_OK;
}

MathStatus_t Vector3_Subtract(const Vector3_t *lhs, const Vector3_t *rhs,
		Vector3_t *result) {

	if ((lhs == NULL) || (rhs == NULL) || result == NULL) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->x = lhs->x - rhs->x;
	result->y = lhs->y - rhs->y;
	result->z = lhs->z - rhs->z;

	return MATH_STATUS_OK;
}

MathStatus_t Vector3_Scale(const Vector3_t *vector, double scale,
		Vector3_t *result) {

	if ((vector == NULL) || (result == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->x = vector->x * scale;
	result->y = vector->y * scale;
	result->z = vector->z * scale;

	return MATH_STATUS_OK;
}

float64_t Vector3_Dot(const Vector3_t *lhs, const Vector3_t *rhs) {
	float64_t result;

	if ((lhs == NULL) || (rhs == NULL)) {
		result = 0.0;
	} else {
		result = (lhs->x * rhs->x) + (lhs->y * rhs->y) + (lhs->z * rhs->z);
	}

	return result;
}

MathStatus_t Vector3_Cross(const Vector3_t *lhs, const Vector3_t *rhs,
		Vector3_t *result) {

	if ((lhs == NULL) || (rhs == NULL) || result == NULL) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->x = (lhs->y * rhs->z) - (lhs->z * rhs->y);
	result->y = (lhs->z * rhs->x) - (lhs->x * rhs->z);
	result->z = (lhs->x * rhs->y) - (lhs->y * rhs->x);

	return MATH_STATUS_OK;
}

float64_t Vector3_Norm(const Vector3_t *vector) {
	float64_t norm_squared;
	float64_t norm;

	if (vector == NULL) {
		norm_squared = 0.0;
	} else {

		norm_squared = (vector->x * vector->x) + (vector->y * vector->y)
				+ (vector->z * vector->z);
	}

	norm = sqrt(norm_squared);

	return norm;
}

MathStatus_t Vector3_Normalize(Vector3_t *vector) {
	float64_t norm;

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
