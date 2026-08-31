/*
 * vector3.h
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef VECTOR3_H_
#define VECTOR3_H_

#include "math_types.h"

MathStatus_t Vector3_Create(double x, double y, double z, Vector3_t *result);

MathStatus_t Vector3_Assign(const Vector3_t *lhs, Vector3_t *result);

MathStatus_t Vector3_Zero(Vector3_t *result);

MathStatus_t Vector3_Add(const Vector3_t *lhs, const Vector3_t *rhs,
		Vector3_t *result);

MathStatus_t Vector3_Subtract(const Vector3_t *lhs, const Vector3_t *rhs,
		Vector3_t *result);

MathStatus_t Vector3_Scale(const Vector3_t *vector, float64_t scale,
		Vector3_t *result);

float64_t Vector3_Dot(const Vector3_t *lhs, const Vector3_t *rhs);

MathStatus_t Vector3_Cross(const Vector3_t *lhs, const Vector3_t *rhs,
		Vector3_t *result);

float64_t Vector3_Norm(const Vector3_t *vector);

MathStatus_t Vector3_Normalize(Vector3_t *vector);

#endif /* VECTOR3_H_ */
