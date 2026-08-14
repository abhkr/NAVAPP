/*
 * dcm.h
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef DCM_H_
#define DCM_H_

#include "math_types.h"

void Dcm_Identity(Matrix3_t *dcm);

void Dcm_FromQuaternion(const Quaternion_t *quaternion, Matrix3_t *dcm);

Quaternion_t Dcm_ToQuaternion(const Matrix3_t *dcm);

void Dcm_MultiplyVector(const Matrix3_t *dcm, const Vector3_t *vector,
		Vector3_t *result);

MathStatus_t Dcm_Orthonormalize(Matrix3_t *dcm);

#endif /* DCM_H_ */
