/*
 * dcm.c
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#include "dcm.h"

#include <math.h>
#include <stdlib.h>

#include "math_constants.h"
#include "quaternion.h"
#include "vector3.h"
#include "matrix.h"

void Dcm_Identity(Matrix3_t *dcm) {
	Matrix3_Identity(dcm);
}

void Dcm_FromQuaternion(const Quaternion_t *quaternion, Matrix3_t *dcm) {

	const double ww = quaternion->w * quaternion->w;
	const double xx = quaternion->x * quaternion->x;
	const double yy = quaternion->y * quaternion->y;
	const double zz = quaternion->z * quaternion->z;

	const double wx = quaternion->w * quaternion->x;
	const double wy = quaternion->w * quaternion->y;
	const double wz = quaternion->w * quaternion->z;

	const double xy = quaternion->x * quaternion->y;
	const double xz = quaternion->x * quaternion->z;
	const double yz = quaternion->y * quaternion->z;

	/*
	 * Quaternion represents NED -> Body frame rotation.
	 *
	 * This matrix is therefore C_b^n in the frame-rotation convention
	 * used by this navigation application.
	 */
	dcm->m00 = ww + xx - yy - zz;
	dcm->m01 = 2.0 * (xy - wz);
	dcm->m02 = 2.0 * (xz + wy);

	dcm->m10 = 2.0 * (xy + wz);
	dcm->m11 = ww - xx + yy - zz;
	dcm->m12 = 2.0 * (yz - wx);

	dcm->m20 = 2.0 * (xz - wy);
	dcm->m21 = 2.0 * (yz + wx);
	dcm->m22 = ww - xx - yy + zz;
}

Quaternion_t Dcm_ToQuaternion(const Matrix3_t *dcm) {
	Quaternion_t quaternion;
	double trace;
	double scale;

	trace = dcm->m00 + dcm->m11 + dcm->m22;

	if (trace > 0.0) {
		scale = sqrt(trace + 1.0) * 2.0;

		quaternion.w = 0.25 * scale;
		quaternion.x = (dcm->m21 - dcm->m12) / scale;
		quaternion.y = (dcm->m02 - dcm->m20) / scale;
		quaternion.z = (dcm->m10 - dcm->m01) / scale;
	} else if ((dcm->m00 > dcm->m11) && (dcm->m00 > dcm->m22)) {
		scale = sqrt(1.0 + dcm->m00 - dcm->m11 - dcm->m22) * 2.0;

		quaternion.w = (dcm->m21 - dcm->m12) / scale;
		quaternion.x = 0.25 * scale;
		quaternion.y = (dcm->m01 + dcm->m10) / scale;
		quaternion.z = (dcm->m02 + dcm->m20) / scale;
	} else if (dcm->m11 > dcm->m22) {
		scale = sqrt(1.0 + dcm->m11 - dcm->m00 - dcm->m22) * 2.0;

		quaternion.w = (dcm->m02 - dcm->m20) / scale;
		quaternion.x = (dcm->m01 + dcm->m10) / scale;
		quaternion.y = 0.25 * scale;
		quaternion.z = (dcm->m12 + dcm->m21) / scale;
	} else {
		scale = sqrt(1.0 + dcm->m22 - dcm->m00 - dcm->m11) * 2.0;

		quaternion.w = (dcm->m10 - dcm->m01) / scale;
		quaternion.x = (dcm->m02 + dcm->m20) / scale;
		quaternion.y = (dcm->m12 + dcm->m21) / scale;
		quaternion.z = 0.25 * scale;
	}

	(void) Quaternion_Normalize(&quaternion);

	return quaternion;
}

void Dcm_MultiplyVector(const Matrix3_t *dcm, const Vector3_t *vector,
		Vector3_t *result) {
	Matrix3_MultiplyVector(dcm, vector, result);
}

MathStatus_t Dcm_Orthonormalize(Matrix3_t *dcm) {
	Vector3_t row0;
	Vector3_t row1;
	Vector3_t row2;
	double error;

	if (dcm == NULL) {
		return MATH_STATUS_NULL_POINTER;
	}

	row0.x = dcm->m00;
	row0.y = dcm->m01;
	row0.z = dcm->m02;

	row1.x = dcm->m10;
	row1.y = dcm->m11;
	row1.z = dcm->m12;

	/*
	 * First-order orthogonalization.
	 */
	error = -0.5 * Vector3_Dot(&row0, &row1);

	row0.x += error * row1.x;
	row0.y += error * row1.y;
	row0.z += error * row1.z;

	row1.x += error * row0.x;
	row1.y += error * row0.y;
	row1.z += error * row0.z;

	row2 = Vector3_Cross(&row0, &row1);

	if ((Vector3_Normalize(&row0) != MATH_STATUS_OK)
			|| (Vector3_Normalize(&row1) != MATH_STATUS_OK)
			|| (Vector3_Normalize(&row2) != MATH_STATUS_OK)) {
		return MATH_STATUS_INVALID_NORM;
	}

	dcm->m00 = row0.x;
	dcm->m01 = row0.y;
	dcm->m02 = row0.z;

	dcm->m10 = row1.x;
	dcm->m11 = row1.y;
	dcm->m12 = row1.z;

	dcm->m20 = row2.x;
	dcm->m21 = row2.y;
	dcm->m22 = row2.z;

	return MATH_STATUS_OK;
}
