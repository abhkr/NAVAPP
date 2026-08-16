/*
 * matrix.h
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include "math_types.h"

MathStatus_t Matrix3_Zero(Matrix3_t *result);

MathStatus_t Matrix3_Identity(Matrix3_t *result);

MathStatus_t Matrix3_Transpose(const Matrix3_t *matrix, Matrix3_t *result);

MathStatus_t Matrix3_Add(const Matrix3_t *lhs, const Matrix3_t *rhs,
		Matrix3_t *result);

MathStatus_t Matrix3_Subtract(const Matrix3_t *lhs, const Matrix3_t *rhs,
		Matrix3_t *result);

MathStatus_t Matrix3_Scale(const Matrix3_t *matrix, double scale,
		Matrix3_t *result);

MathStatus_t Matrix3_Multiply(const Matrix3_t *lhs, const Matrix3_t *rhs,
		Matrix3_t *result);

MathStatus_t Matrix3_MultiplyVector(const Matrix3_t *matrix,
		const Vector3_t *vector, Vector3_t *result);

float64_t Matrix3_Determinant(const Matrix3_t *matrix);

MathStatus_t Matrix3_Inverse(const Matrix3_t *matrix, Matrix3_t *inverse);

#endif /* MATRIX_H_ */
