/*
 * dcm.h
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef DCM_H_
#define DCM_H_

#include "math_types.h"

MathStatus_t Dcm_Identity(Matrix3_t *dcm);

MathStatus_t Dcm_MultiplyVector(const Matrix3_t *dcm, const Vector3_t *vector,
		Vector3_t *result);

MathStatus_t Dcm_Orthonormalize(Matrix3_t *dcm);

#endif /* DCM_H_ */
