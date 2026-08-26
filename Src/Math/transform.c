/*
 * transform.c
 *
 *  Created on: 16-Aug-2026
 *      Author: abhimanyu
 */

#include "transform.h"

#include <math.h>
#include <stddef.h>

#include "transform.h"
#include "dcm.h"
#include "quaternion.h"
#include "math_constants.h"
#include "math_types.h"

#define TRANSFORM_HALF_ANGLE       (0.5)
#define TRANSFORM_SMALL_ANGLE_RAD  (1.0e-10)
#define TRANSFORM_PI               (3.14159265358979323846)
#define TRANSFORM_TWO                (2.0)
#define TRANSFORM_ONE               (1.0)
#define TRANSFORM_NEG_ONE   (-1.0)
#define TRANSFORM_QUATERNION_EPSILON    (1.0e-12)
#define TRANSFORM_SQRT_TWO              (1.41421356237309504880)

static double Transform_Clamp(double value, double minimum, double maximum);

bool Transform_EulerToDcm(const EulerAngles_t *euler, Matrix3_t *dcm) {
	double sin_roll;
	double cos_roll;
	double sin_pitch;
	double cos_pitch;
	double sin_yaw;
	double cos_yaw;

	double roll;
	double pitch;
	double yaw;

	if ((euler == NULL) && (dcm == NULL)) {
		return false;
	}

	roll = euler->roll_rad;
	pitch = euler->pitch_rad;
	yaw = euler->yaw_rad;

	/*
	 * Calculate trigonometric terms once.
	 */
	sin_roll = sin(roll);
	cos_roll = cos(roll);

	sin_pitch = sin(pitch);
	cos_pitch = cos(pitch);

	sin_yaw = sin(yaw);
	cos_yaw = cos(yaw);

	dcm->m00 = cos_pitch * cos_yaw;

	dcm->m01 = cos_pitch * sin_yaw;

	dcm->m02 = -sin_pitch;

	dcm->m10 = (sin_roll * sin_pitch * cos_yaw) - (cos_roll * sin_yaw);

	dcm->m11 = (sin_roll * sin_pitch * sin_yaw) + (cos_roll * cos_yaw);

	dcm->m12 = sin_roll * cos_pitch;

	dcm->m20 = (cos_roll * sin_pitch * cos_yaw) + (sin_roll * sin_yaw);

	dcm->m21 = (cos_roll * sin_pitch * sin_yaw) - (sin_roll * cos_yaw);

	dcm->m22 = cos_roll * cos_pitch;

	return true;
}

bool Transform_DcmToEuler(const Matrix3_t *dcm, EulerAngles_t *euler) {
	double c_theta;
	double s_theta;

	if ((euler == NULL) || (dcm == NULL)) {
		return false;
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
	}
	return true;
}

bool Transform_EulerToQuaternion(const EulerAngles_t *euler,
		Quaternion_t *quaternion) {
	double half_roll;
	double half_pitch;
	double half_yaw;

	double sin_roll;
	double cos_roll;
	double sin_pitch;
	double cos_pitch;
	double sin_yaw;
	double cos_yaw;

	if ((euler == NULL) && (quaternion == NULL)) {
		return false;
	}

	half_roll = TRANSFORM_HALF_ANGLE * euler->roll_rad;

	half_pitch = TRANSFORM_HALF_ANGLE * euler->pitch_rad;

	half_yaw = TRANSFORM_HALF_ANGLE * euler->yaw_rad;

	sin_roll = sin(half_roll);
	cos_roll = cos(half_roll);

	sin_pitch = sin(half_pitch);
	cos_pitch = cos(half_pitch);

	sin_yaw = sin(half_yaw);
	cos_yaw = cos(half_yaw);

	quaternion->w = (cos_roll * cos_pitch * cos_yaw)
			+ (sin_roll * sin_pitch * sin_yaw);

	quaternion->x = (sin_roll * cos_pitch * cos_yaw)
			- (cos_roll * sin_pitch * sin_yaw);

	quaternion->y = (cos_roll * sin_pitch * cos_yaw)
			+ (sin_roll * cos_pitch * sin_yaw);

	quaternion->z = (cos_roll * cos_pitch * sin_yaw)
			- (sin_roll * sin_pitch * cos_yaw);

	Quaternion_Normalize(quaternion);

	return true;
}

bool Transform_QuaternionToEuler(const Quaternion_t *quaternion,
		EulerAngles_t *euler) {
	double q0;
	double q1;
	double q2;
	double q3;

	double quaternion_norm_squared;

	double roll_numerator;
	double roll_denominator;

	double pitch_argument;

	double yaw_numerator;
	double yaw_denominator;

	if ((quaternion == NULL) && (euler == NULL)) {
		return false;
	}

	q0 = quaternion->w;
	q1 = quaternion->x;
	q2 = quaternion->y;
	q3 = quaternion->z;

	/*
	 * Validate quaternion magnitude.
	 */
	quaternion_norm_squared = (q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3);

	if (quaternion_norm_squared <= TRANSFORM_QUATERNION_EPSILON) {
		return false;
	}

	/*
	 * Roll:
	 *
	 * phi =
	 * atan2(
	 *     2(q0q1 + q2q3),
	 *     1 - 2(q1² + q2²)
	 * )
	 */
	roll_numerator = TRANSFORM_TWO * ((q0 * q1) + (q2 * q3));

	roll_denominator = TRANSFORM_ONE - TRANSFORM_TWO * ((q1 * q1) + (q2 * q2));

	/*
	 * Pitch:
	 *
	 * theta =
	 * asin(
	 *     2(q0q2 - q3q1)
	 * )
	 *
	 * Clamp the argument because floating-point
	 * roundoff can produce values slightly outside
	 * [-1, +1].
	 */
	pitch_argument = TRANSFORM_TWO * ((q0 * q2) - (q3 * q1));

	pitch_argument = Transform_Clamp(pitch_argument, -TRANSFORM_ONE,
	TRANSFORM_ONE);

	/*
	 * Yaw:
	 *
	 * psi =
	 * atan2(
	 *     2(q0q3 + q1q2),
	 *     1 - 2(q2² + q3²)
	 * )
	 */
	yaw_numerator = TRANSFORM_TWO * ((q0 * q3) + (q1 * q2));

	yaw_denominator = TRANSFORM_ONE - TRANSFORM_TWO * ((q2 * q2) + (q3 * q3));

	euler->roll_rad = atan2(roll_numerator, roll_denominator);

	euler->pitch_rad = asin(pitch_argument);

	euler->yaw_rad = atan2(yaw_numerator, yaw_denominator);

	return true;
}

bool Transform_DcmToQuaternion(const Matrix3_t *dcm, Quaternion_t *quaternion) {
	float64_t trace;
	float64_t scale;
	float64_t w;
	float64_t x;
	float64_t y;
	float64_t z;

	if ((dcm == NULL) || (quaternion == NULL)) {
		return false;
	}

	trace = dcm->m00 + dcm->m11 + dcm->m22;

	/*
	 * The trace is:
	 *
	 * trace(C) = 1 + 2*cos(theta)
	 *
	 * When the trace is positive, calculating the scalar
	 * component first gives good numerical accuracy.
	 */
	if (trace > 0.0) {
		scale = 2.0 * sqrt(trace + 1.0);

		if (scale < TRANSFORM_QUATERNION_EPSILON) {
			return false;
		}

		w = 0.25 * scale;

		x = (dcm->m12 - dcm->m21) / scale;

		y = (dcm->m20 - dcm->m02) / scale;

		z = (dcm->m01 - dcm->m10) / scale;
	} else if ((dcm->m00 > dcm->m11) && (dcm->m00 > dcm->m22)) {
		/*
		 * X diagonal element is largest.
		 */
		scale = 2.0 * sqrt(1.0 + dcm->m00 - dcm->m11 - dcm->m22);

		if (scale < TRANSFORM_QUATERNION_EPSILON) {
			return false;
		}

		x = 0.25 * scale;

		w = (dcm->m12 - dcm->m21) / scale;

		y = (dcm->m01 + dcm->m10) / scale;

		z = (dcm->m02 + dcm->m20) / scale;
	} else if (dcm->m11 > dcm->m22) {
		/*
		 * Y diagonal element is largest.
		 */
		scale = 2.0 * sqrt(1.0 + dcm->m11 - dcm->m00 - dcm->m22);

		if (scale < TRANSFORM_QUATERNION_EPSILON) {
			return false;
		}

		y = 0.25 * scale;

		w = (dcm->m20 - dcm->m02) / scale;

		x = (dcm->m01 + dcm->m10) / scale;

		z = (dcm->m12 + dcm->m21) / scale;
	} else {
		/*
		 * Z diagonal element is largest.
		 */
		scale = 2.0 * sqrt(1.0 + dcm->m22 - dcm->m00 - dcm->m11);

		if (scale < TRANSFORM_QUATERNION_EPSILON) {
			return false;
		}

		z = 0.25 * scale;

		w = (dcm->m01 - dcm->m10) / scale;

		x = (dcm->m02 + dcm->m20) / scale;

		y = (dcm->m12 + dcm->m21) / scale;
	}

	/*
	 * Store the result only after the calculation has
	 * completed successfully.
	 */
	quaternion->w = w;
	quaternion->x = x;
	quaternion->y = y;
	quaternion->z = z;

	/*
	 * Quaternion q and -q represent the same rotation.
	 *
	 * Select a deterministic representation so that the
	 * scalar component is non-negative.
	 */
	if (quaternion->w < 0.0) {
		quaternion->w = -quaternion->w;
		quaternion->x = -quaternion->x;
		quaternion->y = -quaternion->y;
		quaternion->z = -quaternion->z;
	}

	/*
	 * Floating-point errors in the input DCM can cause the
	 * quaternion magnitude to deviate slightly from unity.
	 */
	if (Quaternion_Normalize(quaternion) != MATH_STATUS_OK) {
		return false;
	}

	return true;
}

bool Transform_QuaternionToDcm(const Quaternion_t *quaternion, Matrix3_t *dcm) {
	double q0;
	double q1;
	double q2;
	double q3;

	double q0q1;
	double q0q2;
	double q0q3;

	double q1q1;
	double q1q2;
	double q1q3;

	double q2q2;
	double q2q3;

	double q3q3;

	double quaternion_norm_squared;

	if ((quaternion == NULL) && (dcm == NULL)) {
		return false;
	}

	q0 = quaternion->w;
	q1 = quaternion->x;
	q2 = quaternion->y;
	q3 = quaternion->z;

	quaternion_norm_squared = (q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3);

	if (quaternion_norm_squared <= TRANSFORM_QUATERNION_EPSILON) {
		return false;
	}

	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;

	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;

	q2q2 = q2 * q2;
	q2q3 = q2 * q3;

	q3q3 = q3 * q3;

	dcm->m00 = TRANSFORM_ONE - TRANSFORM_TWO * (q2q2 + q3q3);

	dcm->m01 = TRANSFORM_TWO * (q1q2 + q0q3);

	dcm->m02 = TRANSFORM_TWO * (q1q3 - q0q2);

	dcm->m10 = TRANSFORM_TWO * (q1q2 - q0q3);

	dcm->m11 = TRANSFORM_ONE - TRANSFORM_TWO * (q1q1 + q3q3);

	dcm->m12 = TRANSFORM_TWO * (q2q3 + q0q1);

	dcm->m20 = TRANSFORM_TWO * (q1q3 + q0q2);

	dcm->m21 = TRANSFORM_TWO * (q2q3 - q0q1);

	dcm->m22 = TRANSFORM_ONE - TRANSFORM_TWO * (q1q1 + q2q2);

	return true;
}

//bool Transform_EulerToQuaternion(const EulerAngles_t *euler,
//		Quaternion_t *quaternion) {
//	Matrix3_t dcm;
//
//	if ((euler == NULL) || (quaternion == NULL)) {
//		return false;
//	}
//
//	if (Euler_ToDcm(euler, &dcm) == false) {
//		return false;
//	}
//
//	return Dcm_ToQuaternion(&dcm, quaternion);
//}
//
//bool Transform_DcmToEuler(const Matrix3_t *dcm, EulerAngles_t *euler) {
//	if ((dcm == NULL) || (euler == NULL)) {
//		return false;
//	}
//
//	return Dcm_ToEuler(dcm, euler);
//}
//
//bool Transform_EulerToDcm(const EulerAngles_t *euler, Matrix3_t *dcm) {
//	if ((euler == NULL) || (dcm == NULL)) {
//		return false;
//	}
//
//	return Euler_ToDcm(euler, dcm);
//}
//
//bool Transform_RotationVectorToQuaternion(const Vector3_t *rotation_vector,
//		Quaternion_t *quaternion) {
//	double angle;
//	double half_angle;
//	double scale;
//
//	if ((rotation_vector == NULL) || (quaternion == NULL)) {
//		return false;
//	}
//
//	angle = sqrt(
//			(rotation_vector->x * rotation_vector->x)
//					+ (rotation_vector->y * rotation_vector->y)
//					+ (rotation_vector->z * rotation_vector->z));
//
//	if (angle < TRANSFORM_SMALL_ANGLE_RAD) {
//		/*
//		 * Small-angle approximation.
//		 */
//		quaternion->w = 1.0;
//		quaternion->x =
//		TRANSFORM_HALF_ANGLE * rotation_vector->x;
//		quaternion->y =
//		TRANSFORM_HALF_ANGLE * rotation_vector->y;
//		quaternion->z =
//		TRANSFORM_HALF_ANGLE * rotation_vector->z;
//
//		return Quaternion_Normalize(quaternion);
//	}
//
//	half_angle =
//	TRANSFORM_HALF_ANGLE * angle;
//
//	scale = sin(half_angle) / angle;
//
//	quaternion->w = cos(half_angle);
//
//	quaternion->x = scale * rotation_vector->x;
//
//	quaternion->y = scale * rotation_vector->y;
//
//	quaternion->z = scale * rotation_vector->z;
//
//	return Quaternion_Normalize(quaternion);
//}
//
//bool Transform_QuaternionToRotationVector(const Quaternion_t *quaternion,
//		Vector3_t *rotation_vector) {
//	Quaternion_t normalized_quaternion;
//
//	double angle;
//	double sin_half_angle;
//	double scale;
//
//	if ((quaternion == NULL) || (rotation_vector == NULL)) {
//		return false;
//	}
//
//	normalized_quaternion = *quaternion;
//
//	if (Quaternion_Normalize(&normalized_quaternion) == false) {
//		return false;
//	}
//
//	/*
//	 * Clamp prevents acos() domain errors due to
//	 * floating-point roundoff.
//	 */
//	normalized_quaternion.w = Transform_Clamp(normalized_quaternion.w, -1.0,
//			1.0);
//
//	angle = 2.0 * acos(normalized_quaternion.w);
//
//	sin_half_angle = sqrt(
//			(normalized_quaternion.x * normalized_quaternion.x)
//					+ (normalized_quaternion.y * normalized_quaternion.y)
//					+ (normalized_quaternion.z * normalized_quaternion.z));
//
//	if (sin_half_angle <
//	TRANSFORM_SMALL_ANGLE_RAD) {
//		rotation_vector->x = 2.0 * normalized_quaternion.x;
//
//		rotation_vector->y = 2.0 * normalized_quaternion.y;
//
//		rotation_vector->z = 2.0 * normalized_quaternion.z;
//
//		return true;
//	}
//
//	scale = angle / sin_half_angle;
//
//	rotation_vector->x = scale * normalized_quaternion.x;
//
//	rotation_vector->y = scale * normalized_quaternion.y;
//
//	rotation_vector->z = scale * normalized_quaternion.z;
//
//	return true;
//}
//
//bool Transform_RotationVectorToDcm(const Vector3_t *rotation_vector,
//		Matrix3_t *dcm) {
//	Quaternion_t quaternion;
//
//	if ((rotation_vector == NULL) || (dcm == NULL)) {
//		return false;
//	}
//
//	if (Transform_RotationVectorToQuaternion(rotation_vector,
//			&quaternion) == false) {
//		return false;
//	}
//
//	return Transform_QuaternionToDcm(&quaternion, dcm);
//}
//
//bool Transform_DcmToRotationVector(const Matrix3_t *dcm,
//		Vector3_t *rotation_vector) {
//	Quaternion_t quaternion;
//
//	if ((dcm == NULL) || (rotation_vector == NULL)) {
//		return false;
//	}
//
//	if (Transform_DcmToQuaternion(dcm, &quaternion) == false) {
//		return false;
//	}
//
//	return Transform_QuaternionToRotationVector(&quaternion, rotation_vector);
//}
//
//static bool Transform_IsValidDcm(const Matrix3_t *dcm) {
//	/*
//	 * DCM validation should eventually use the existing
//	 * matrix orthogonality/determinant functions.
//	 */
//	(void) dcm;
//
//	return true;
//}

static double Transform_Clamp(double value, double minimum, double maximum) {
	double result;

	result = value;

	if (value < minimum) {
		result = minimum;
	} else if (value > maximum) {
		result = maximum;
	}

	return result;
}
