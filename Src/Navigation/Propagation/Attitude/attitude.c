/*
 * attitude.c
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#include "attitude.h"

#include "dcm.h"
#include "quaternion.h"

void Attitude_Init(NavigationAttitude_t *attitude) {
	if (attitude != NULL) {
		Quaternion_Identity(&attitude->q_ned_to_body);

		Dcm_Identity(&attitude->dcm_ned_to_body);

		attitude->roll_rad = 0.0;
		attitude->pitch_rad = 0.0;
		attitude->yaw_rad = 0.0;
	}
}

void Attitude_Propagate(NavigationAttitude_t *attitude,
		const NavigationImuIncrement_t *imu) {
	if ((attitude != NULL) && (imu != NULL) && (imu->valid == true)) {
//		Quaternion_UpdateFromDeltaAngle(&attitude->q_ned_to_body,
//				&imu->delta_angle_rad);
//
//		Quaternion_Normalize(&attitude->q_ned_to_body);
//
//		Dcm_FromQuaternion(&attitude->dcm_ned_to_body,
//				&attitude->q_ned_to_body);
//
//		Euler_FromDcm(&attitude->dcm_ned_to_body, &attitude->roll_rad,
//				&attitude->pitch_rad, &attitude->yaw_rad);
	}
}
