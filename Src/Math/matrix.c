/*
 * matrix.c
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#include "matrix.h"

#include "math_constants.h"

#include <math.h>
#include <stdlib.h>

MathStatus_t Matrix3_Zero(Matrix3_t *result) {

	if (result == NULL) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->m00 = 0.0;
	result->m01 = 0.0;
	result->m02 = 0.0;

	result->m10 = 0.0;
	result->m11 = 0.0;
	result->m12 = 0.0;

	result->m20 = 0.0;
	result->m21 = 0.0;
	result->m22 = 0.0;

	return MATH_STATUS_OK;
}

MathStatus_t Matrix3_Identity(Matrix3_t *result) {

	if (result == NULL) {
		return MATH_STATUS_NULL_POINTER;
	}

	Matrix3_Zero(result);

	result->m00 = 1.0;
	result->m11 = 1.0;
	result->m22 = 1.0;

	return MATH_STATUS_OK;
}

MathStatus_t Matrix3_Transpose(const Matrix3_t *matrix, Matrix3_t *result) {
	Matrix3_t temp;
	uint8_t i;
	uint8_t j;

	if ((matrix == NULL) || (result == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}

	temp.m00 = matrix->m00;
	temp.m01 = matrix->m10;
	temp.m02 = matrix->m20;

	temp.m10 = matrix->m01;
	temp.m11 = matrix->m11;
	temp.m12 = matrix->m21;

	temp.m20 = matrix->m02;
	temp.m21 = matrix->m12;
	temp.m22 = matrix->m22;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			result->m_data[i][j] = temp.m_data[i][j];
		}
	}

	return MATH_STATUS_OK;
}

MathStatus_t Matrix3_Add(const Matrix3_t *lhs, const Matrix3_t *rhs,
		Matrix3_t *result) {

	if ((lhs == NULL) || (rhs == NULL) || (result == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->m00 = lhs->m00 + rhs->m00;
	result->m01 = lhs->m01 + rhs->m01;
	result->m02 = lhs->m02 + rhs->m02;

	result->m10 = lhs->m10 + rhs->m10;
	result->m11 = lhs->m11 + rhs->m11;
	result->m12 = lhs->m12 + rhs->m12;

	result->m20 = lhs->m20 + rhs->m20;
	result->m21 = lhs->m21 + rhs->m21;
	result->m22 = lhs->m22 + rhs->m22;

	return MATH_STATUS_OK;
}

MathStatus_t Matrix3_Subtract(const Matrix3_t *lhs, const Matrix3_t *rhs,
		Matrix3_t *result) {

	if ((lhs == NULL) || (rhs == NULL) || (result == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->m00 = lhs->m00 - rhs->m00;
	result->m01 = lhs->m01 - rhs->m01;
	result->m02 = lhs->m02 - rhs->m02;

	result->m10 = lhs->m10 - rhs->m10;
	result->m11 = lhs->m11 - rhs->m11;
	result->m12 = lhs->m12 - rhs->m12;

	result->m20 = lhs->m20 - rhs->m20;
	result->m21 = lhs->m21 - rhs->m21;
	result->m22 = lhs->m22 - rhs->m22;

	return MATH_STATUS_OK;
}

MathStatus_t Matrix3_Scale(const Matrix3_t *matrix, double scale,
		Matrix3_t *result) {

	if ((matrix == NULL) || (result == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->m00 = matrix->m00 * scale;
	result->m01 = matrix->m01 * scale;
	result->m02 = matrix->m02 * scale;

	result->m10 = matrix->m10 * scale;
	result->m11 = matrix->m11 * scale;
	result->m12 = matrix->m12 * scale;

	result->m20 = matrix->m20 * scale;
	result->m21 = matrix->m21 * scale;
	result->m22 = matrix->m22 * scale;

	return MATH_STATUS_OK;
}

MathStatus_t Matrix3_Multiply(const Matrix3_t *lhs, const Matrix3_t *rhs,
		Matrix3_t *result) {

	if ((lhs == NULL) || (rhs == NULL) || (result == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->m00 = (lhs->m00 * rhs->m00) + (lhs->m01 * rhs->m10)
			+ (lhs->m02 * rhs->m20);

	result->m01 = (lhs->m00 * rhs->m01) + (lhs->m01 * rhs->m11)
			+ (lhs->m02 * rhs->m21);

	result->m02 = (lhs->m00 * rhs->m02) + (lhs->m01 * rhs->m12)
			+ (lhs->m02 * rhs->m22);

	result->m10 = (lhs->m10 * rhs->m00) + (lhs->m11 * rhs->m10)
			+ (lhs->m12 * rhs->m20);

	result->m11 = (lhs->m10 * rhs->m01) + (lhs->m11 * rhs->m11)
			+ (lhs->m12 * rhs->m21);

	result->m12 = (lhs->m10 * rhs->m02) + (lhs->m11 * rhs->m12)
			+ (lhs->m12 * rhs->m22);

	result->m20 = (lhs->m20 * rhs->m00) + (lhs->m21 * rhs->m10)
			+ (lhs->m22 * rhs->m20);

	result->m21 = (lhs->m20 * rhs->m01) + (lhs->m21 * rhs->m11)
			+ (lhs->m22 * rhs->m21);

	result->m22 = (lhs->m20 * rhs->m02) + (lhs->m21 * rhs->m12)
			+ (lhs->m22 * rhs->m22);

	return MATH_STATUS_OK;
}

MathStatus_t Matrix3_MultiplyVector(const Matrix3_t *matrix,
		const Vector3_t *vector, Vector3_t *result) {

	if ((matrix == NULL) || (vector == NULL) || (result == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}

	result->x = (matrix->m00 * vector->x) + (matrix->m01 * vector->y)
			+ (matrix->m02 * vector->z);

	result->y = (matrix->m10 * vector->x) + (matrix->m11 * vector->y)
			+ (matrix->m12 * vector->z);

	result->z = (matrix->m20 * vector->x) + (matrix->m21 * vector->y)
			+ (matrix->m22 * vector->z);

	return MATH_STATUS_OK;
}

float64_t Matrix3_Determinant(const Matrix3_t *matrix) {
	float64_t determinant;

	if (matrix == NULL) {
		determinant = 0.0;
	} else {

		determinant = (matrix->m00
				* ((matrix->m11 * matrix->m22) - (matrix->m12 * matrix->m21)))
				- (matrix->m01
						* ((matrix->m10 * matrix->m22)
								- (matrix->m12 * matrix->m20)))
				+ (matrix->m02
						* ((matrix->m10 * matrix->m21)
								- (matrix->m11 * matrix->m20)));
	}

	return determinant;
}

MathStatus_t Matrix3_Inverse(const Matrix3_t *matrix, Matrix3_t *inverse) {
	double determinant;
	double inverse_determinant;

	if ((matrix == NULL) || (inverse == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}

	determinant = Matrix3_Determinant(matrix);

	if (fabs(determinant) <= MATH_EPSILON) {
		return MATH_STATUS_SINGULAR_MATRIX;
	}

	inverse_determinant = 1.0 / determinant;

	inverse->m00 = ((matrix->m11 * matrix->m22) - (matrix->m12 * matrix->m21))
			* inverse_determinant;

	inverse->m01 = ((matrix->m02 * matrix->m21) - (matrix->m01 * matrix->m22))
			* inverse_determinant;

	inverse->m02 = ((matrix->m01 * matrix->m12) - (matrix->m02 * matrix->m11))
			* inverse_determinant;

	inverse->m10 = ((matrix->m12 * matrix->m20) - (matrix->m10 * matrix->m22))
			* inverse_determinant;

	inverse->m11 = ((matrix->m00 * matrix->m22) - (matrix->m02 * matrix->m20))
			* inverse_determinant;

	inverse->m12 = ((matrix->m02 * matrix->m10) - (matrix->m00 * matrix->m12))
			* inverse_determinant;

	inverse->m20 = ((matrix->m10 * matrix->m21) - (matrix->m11 * matrix->m20))
			* inverse_determinant;

	inverse->m21 = ((matrix->m01 * matrix->m20) - (matrix->m00 * matrix->m21))
			* inverse_determinant;

	inverse->m22 = ((matrix->m00 * matrix->m11) - (matrix->m01 * matrix->m10))
			* inverse_determinant;

	return MATH_STATUS_OK;
}
