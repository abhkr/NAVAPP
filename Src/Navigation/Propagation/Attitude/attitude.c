/*
 * attitude.c
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#include <stdlib.h>
#include <stdbool.h>

#include "attitude.h"
#include "dcm.h"
#include "quaternion.h"

static void Coning_Compensate_4Sample(const ImuMeasurement_t imu_samples[4],
		ImuMeasurement_t *corrected);

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

static void Coning_Compensate_4Sample(const ImuMeasurement_t samples[4],
		ImuMeasurement_t *corrected) {
	Vector3_t dth01;
	Vector3_t dth12;
	Vector3_t dth23;
	Vector3_t dth02;
	Vector3_t dth13;
	Vector3_t dth03;

	Vector3_Cross(&samples[0].gyro_rad_delt, &samples[1].gyro_rad_delt, &dth01);
	Vector3_Cross(&samples[1].gyro_rad_delt, &samples[2].gyro_rad_delt, &dth12);
	Vector3_Cross(&samples[2].gyro_rad_delt, &samples[3].gyro_rad_delt, &dth23);
	Vector3_Cross(&samples[0].gyro_rad_delt, &samples[2].gyro_rad_delt, &dth02);
	Vector3_Cross(&samples[1].gyro_rad_delt, &samples[3].gyro_rad_delt, &dth13);
	Vector3_Cross(&samples[0].gyro_rad_delt, &samples[3].gyro_rad_delt, &dth03);

}
