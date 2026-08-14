/*
 * euler.h
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef EULER_H_
#define EULER_H_

#include "math_types.h"

void EulerAngles_Zero(EulerAngles_t *euler);

void Euler_FromQuaternion(const Quaternion_t *quaternion, EulerAngles_t *euler);

void Euler_ToQuaternion(const EulerAngles_t *euler, Quaternion_t *quaternion);

void Euler_FromDcm_stp(const Matrix3_t *dcm, EulerAngles_t *euler);

void Euler_ToDcm_stp(const EulerAngles_t *euler, Matrix3_t *dcm);

#endif /* EULER_H_ */
