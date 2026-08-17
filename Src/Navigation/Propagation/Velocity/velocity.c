/*
 * velocity.c
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#include <stdlib.h>
#include <stdbool.h>

#include "velocity.h"

static Vector3_t Velocity_TransformBodyToNed(
		const NavigationAttitude_t *attitude, const Vector3_t *body_vector);

void Velocity_Init(NavigationVelocity_t *velocity) {
	if (velocity != NULL) {
		velocity->north_m_s = 0.0;
		velocity->east_m_s = 0.0;
		velocity->down_m_s = 0.0;
	}
}

void Velocity_Propagate(NavigationVelocity_t *velocity,
		const NavigationAttitude_t *attitude,
		const NavigationGravity_t *gravity, const NavigationImuIncrement_t *imu) {
	Vector3_t delta_velocity_ned;

	if ((velocity != NULL) && (attitude != NULL) && (gravity != NULL)
			&& (imu != NULL) && (imu->valid == true)) {
		delta_velocity_ned = Velocity_TransformBodyToNed(attitude,
				&imu->delta_velocity_m_s);

		/*
		 * Specific force contribution.
		 */
		velocity->north_m_s += delta_velocity_ned.x;

		velocity->east_m_s += delta_velocity_ned.y;

		/*
		 * NED convention:
		 * gravity acts in positive Down.
		 */
		velocity->down_m_s += delta_velocity_ned.z
				+ (gravity->gravity_ned_m_s2.z * imu->dt_s);
	}
}

static Vector3_t Velocity_TransformBodyToNed(
		const NavigationAttitude_t *attitude, const Vector3_t *body_vector) {
	Vector3_t ned_vector;

	ned_vector.x = (attitude->dcm_ned_to_body.m00 * body_vector->x)
			+ (attitude->dcm_ned_to_body.m10 * body_vector->y)
			+ (attitude->dcm_ned_to_body.m20 * body_vector->z);

	ned_vector.y = (attitude->dcm_ned_to_body.m01 * body_vector->x)
			+ (attitude->dcm_ned_to_body.m11 * body_vector->y)
			+ (attitude->dcm_ned_to_body.m21 * body_vector->z);

	ned_vector.z = (attitude->dcm_ned_to_body.m02 * body_vector->x)
			+ (attitude->dcm_ned_to_body.m12 * body_vector->y)
			+ (attitude->dcm_ned_to_body.m22 * body_vector->z);

	return ned_vector;
}
