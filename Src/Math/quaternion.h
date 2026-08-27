/*
 * quaternion.h
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include "math_types.h"

MathStatus_t Quaternion_Identity(Quaternion_t *result);

MathStatus_t Quaternion_Create(float64_t w, float64_t x, float64_t y,
		float64_t z, Quaternion_t *result);

MathStatus_t Quaternion_Multiply(const Quaternion_t *lhs,
		const Quaternion_t *rhs, Quaternion_t *result);

float64_t Quaternion_Norm(const Quaternion_t *quaternion);

MathStatus_t Quaternion_Normalize(Quaternion_t *quaternion);

MathStatus_t Quaternion_Conjugate(const Quaternion_t *quaternion,
		Quaternion_t *result);

MathStatus_t Quaternion_Inverse(const Quaternion_t *quaternion,
		Quaternion_t *result);

MathStatus_t Quaternion_FromRotationVector(const Vector3_t *rotation_vector_rad,
		Quaternion_t *result);

MathStatus_t Quaternion_Midpoint(const Quaternion_t *q0, const Quaternion_t *q1,
		Quaternion_t *q_mid);

#endif /* QUATERNION_H_ */
