/*
 * euler.c
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#include "euler.h"

#include <math.h>
#include <stdlib.h>

#include "dcm.h"
#include "math_constants.h"
#include "quaternion.h"

void EulerAngles_Zero(EulerAngles_t *euler) {

	euler->yaw_rad = 0.0;
	euler->roll_rad = 0.0;
	euler->pitch_rad = 0.0;
}

void Euler_FromQuaternion(const Quaternion_t *quaternion, EulerAngles_t *euler) {
	double sin_pitch;

	sin_pitch =
			2.0
					* ((quaternion->w * quaternion->y)
							- (quaternion->z * quaternion->x));

	if (sin_pitch > 1.0) {
		sin_pitch = 1.0;
	} else if (sin_pitch < -1.0) {
		sin_pitch = -1.0;
	} else {
		/* Value already within valid range. */
	}

	euler->roll_rad = atan2(
			2.0
					* ((quaternion->w * quaternion->x)
							+ (quaternion->y * quaternion->z)),
			1.0
					- 2.0
							* ((quaternion->x * quaternion->x)
									+ (quaternion->y * quaternion->y)));

	euler->pitch_rad = asin(sin_pitch);

	euler->yaw_rad = atan2(
			2.0
					* ((quaternion->w * quaternion->z)
							+ (quaternion->x * quaternion->y)),
			1.0
					- 2.0
							* ((quaternion->y * quaternion->y)
									+ (quaternion->z * quaternion->z)));
}

void Euler_ToQuaternion(const EulerAngles_t *euler, Quaternion_t *quaternion) {

	const double half_roll = 0.5 * euler->roll_rad;
	const double half_pitch = 0.5 * euler->pitch_rad;
	const double half_yaw = 0.5 * euler->yaw_rad;

	const double cr = cos(half_roll);
	const double sr = sin(half_roll);

	const double cp = cos(half_pitch);
	const double sp = sin(half_pitch);

	const double cy = cos(half_yaw);
	const double sy = sin(half_yaw);

	quaternion->w = (cr * cp * cy) + (sr * sp * sy);
	quaternion->x = (sr * cp * cy) - (cr * sp * sy);
	quaternion->y = (cr * sp * cy) + (sr * cp * sy);
	quaternion->z = (cr * cp * sy) - (sr * sp * cy);
}

MathStatus_t Euler_FromDcm_stp(const Matrix3_t *dcm, EulerAngles_t *euler) {
	double c_theta;
	double s_theta;

	if ((euler == NULL) || (dcm == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}

	s_theta = -dcm->m02;

	if (s_theta > 1.0) {
		s_theta = 1.0;
	} else if (s_theta < -1.0) {
		s_theta = -1.0;
	} else {
		/* Value is already within the valid range. */
	}
	euler->pitch_rad = asin(s_theta);

	c_theta = cos(euler->pitch_rad);

	if (c_theta > MATH_EPSILON) {
		euler->roll_rad = atan2(dcm->m12, dcm->m22);
		euler->yaw_rad = atan2(dcm->m01, dcm->m00);
	} else {
		euler->roll_rad = 0.0;
		if (s_theta >= 0.0) {
			euler->yaw_rad = atan2(-dcm->m10, dcm->m11);
		} else {
			euler->yaw_rad = atan2(dcm->m10, dcm->m11);
		}
		return MATH_STATUS_SINGULAR_MATRIX;
	}

	return MATH_STATUS_OK;
}

MathStatus_t Euler_ToDcm_stp(const EulerAngles_t *euler, Matrix3_t *dcm) {
	double c_psi;
	double s_psi;
	double c_phi;
	double s_phi;
	double c_theta;
	double s_theta;

	if ((euler == NULL) || (dcm == NULL)) {
		return MATH_STATUS_NULL_POINTER;
	}

	c_psi = cos(euler->yaw_rad);
	s_psi = sin(euler->yaw_rad);

	c_phi = cos(euler->roll_rad);
	s_phi = sin(euler->roll_rad);

	c_theta = cos(euler->pitch_rad);
	s_theta = sin(euler->pitch_rad);

	dcm->m00 = c_theta * c_psi;
	dcm->m01 = c_theta * s_psi;
	dcm->m02 = -s_theta;

	dcm->m10 = (c_psi * s_theta * s_phi) - (c_phi * s_psi);
	dcm->m11 = (c_phi * c_psi) + (s_theta * s_phi * s_psi);
	dcm->m12 = c_theta * s_phi;

	dcm->m20 = (c_phi * c_psi * s_theta) + (s_phi * s_psi);
	dcm->m21 = (-c_psi * s_phi) + (c_phi * s_theta * s_psi);
	dcm->m22 = c_theta * c_phi;

	return MATH_STATUS_OK;
}
