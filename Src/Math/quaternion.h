/*
 * quaternion.h
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include "math_types.h"

void Quaternion_Identity(Quaternion_t *result);

Quaternion_t Quaternion_Create(double w, double x, double y, double z);

Quaternion_t Quaternion_Multiply(const Quaternion_t *lhs,
		const Quaternion_t *rhs);

double Quaternion_Norm(const Quaternion_t *quaternion);

MathStatus_t Quaternion_Normalize(Quaternion_t *quaternion);

Quaternion_t Quaternion_Conjugate(const Quaternion_t *quaternion);

Quaternion_t Quaternion_Inverse(const Quaternion_t *quaternion);

Quaternion_t Quaternion_FromRotationVector(const Vector3_t *rotation_vector_rad);

#endif /* QUATERNION_H_ */
