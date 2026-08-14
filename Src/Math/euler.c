/*
 * euler.c
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#include "euler.h"

#include <math.h>

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

void Euler_FromDcm_stp(const Matrix3_t *dcm, EulerAngles_t *euler) {
	Quaternion_t quaternion;

	quaternion = Dcm_ToQuaternion(dcm);

	Euler_FromQuaternion(&quaternion, euler);
}

void Euler_ToDcm_stp(const EulerAngles_t *euler, Matrix3_t *dcm) {
	Quaternion_t quaternion;

	Euler_ToQuaternion(euler, &quaternion);

	Dcm_FromQuaternion(&quaternion, dcm);
}
