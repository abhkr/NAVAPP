/*
 * transform.h
 *
 *  Created on: 16-Aug-2026
 *      Author: abhimanyu
 */

#ifndef MATH_TRANSFORM_H_
#define MATH_TRANSFORM_H_

#include <stdbool.h>

#include "math_types.h"

/**
 * @brief Convert Euler angles to DCM.
 *
 * @param euler Input Euler angles.
 * @param dcm Output DCM.
 *
 * @return true if conversion is successful.
 */
bool Transform_EulerToDcm(const EulerAngles_t *euler, Matrix3_t *dcm);

/**
 * @brief Convert DCM to Euler angles.
 *
 * @param dcm Input DCM.
 * @param euler Output Euler angles.
 *
 * @return true if conversion is successful.
 */
bool Transform_DcmToEuler(const Matrix3_t *dcm, EulerAngles_t *euler);

/**
 * @brief Convert quaternion to direction cosine matrix.
 *
 * @param quaternion Input quaternion.
 * @param dcm Output DCM.
 *
 * @return true if conversion is successful.
 */
bool Transform_QuaternionToDcm(const Quaternion_t *quaternion, Matrix3_t *dcm);

/**
 * @brief Convert direction cosine matrix to quaternion.
 *
 * @param dcm Input DCM.
 * @param quaternion Output quaternion.
 *
 * @return true if conversion is successful.
 */
bool Transform_DcmToQuaternion(const Matrix3_t *dcm, Quaternion_t *quaternion);

/**
 * @brief Convert quaternion to Euler angles.
 *
 * @param quaternion Input quaternion.
 * @param euler Output Euler angles.
 *
 * @return true if conversion is successful.
 */
bool Transform_QuaternionToEuler(const Quaternion_t *quaternion,
		EulerAngles_t *euler);

/**
 * @brief Convert Euler angles to quaternion.
 *
 * @param euler Input Euler angles.
 * @param quaternion Output quaternion.
 *
 * @return true if conversion is successful.
 */
bool Transform_EulerToQuaternion(const EulerAngles_t *euler,
		Quaternion_t *quaternion);

/**
 * @brief Convert rotation vector to quaternion.
 *
 * @param rotation_vector Rotation vector [rad].
 * @param quaternion Output quaternion.
 *
 * @return true if conversion is successful.
 */
bool Transform_RotationVectorToQuaternion(const Vector3_t *rotation_vector,
		Quaternion_t *quaternion);

/**
 * @brief Convert quaternion to rotation vector.
 *
 * @param quaternion Input quaternion.
 * @param rotation_vector Output rotation vector [rad].
 *
 * @return true if conversion is successful.
 */
bool Transform_QuaternionToRotationVector(const Quaternion_t *quaternion,
		Vector3_t *rotation_vector);

/**
 * @brief Convert rotation vector to DCM.
 *
 * @param rotation_vector Rotation vector [rad].
 * @param dcm Output DCM.
 *
 * @return true if conversion is successful.
 */
bool Transform_RotationVectorToDcm(const Vector3_t *rotation_vector,
		Matrix3_t *dcm);

/**
 * @brief Convert DCM to rotation vector.
 *
 * @param dcm Input DCM.
 * @param rotation_vector Output rotation vector [rad].
 *
 * @return true if conversion is successful.
 */
bool Transform_DcmToRotationVector(const Matrix3_t *dcm,
		Vector3_t *rotation_vector);

#endif /* MATH_TRANSFORM_H_ */
