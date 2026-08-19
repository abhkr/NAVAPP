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
#include "imu_types.h"

void Coning_Compensate_4Sample(const ImuMeasurement_t imu_samples[4],
		ImuMeasurement_t *corrected);

static void Sculling_Compensate_4Sample(const ImuMeasurement_t samples[4],
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

void Coning_Compensate_4Sample(const ImuMeasurement_t samples[4],
		ImuMeasurement_t *corrected) {
	Vector3_t dth01;
	Vector3_t dth12;
	Vector3_t dth23;
	Vector3_t dth02;
	Vector3_t dth13;
	Vector3_t dth03;
	Vector3_t temp;

	Vector3_Cross(&samples[0].gyro_rad_delt, &samples[1].gyro_rad_delt, &dth01);
	Vector3_Cross(&samples[1].gyro_rad_delt, &samples[2].gyro_rad_delt, &dth12);
	Vector3_Cross(&samples[2].gyro_rad_delt, &samples[3].gyro_rad_delt, &dth23);
	Vector3_Cross(&samples[0].gyro_rad_delt, &samples[2].gyro_rad_delt, &dth02);
	Vector3_Cross(&samples[1].gyro_rad_delt, &samples[3].gyro_rad_delt, &dth13);
	Vector3_Cross(&samples[0].gyro_rad_delt, &samples[3].gyro_rad_delt, &dth03);

	Vector3_Scale(&dth03, CONE_K1, &corrected->gyro_rad_delt);

	Vector3_Add(&dth02, &dth13, &temp);
	Vector3_Scale(&temp, CONE_K2, &temp);

	Vector3_Add(&corrected->gyro_rad_delt, &temp, &corrected->gyro_rad_delt);

	Vector3_Add(&dth01, &dth12, &temp);
	Vector3_Add(&temp, &dth23, &temp);
	Vector3_Scale(&temp, CONE_K3, &temp);

	Vector3_Add(&corrected->gyro_rad_delt, &temp, &corrected->gyro_rad_delt);

	Vector3_Add(&corrected->gyro_rad_delt, &samples[0].gyro_rad_delt,
			&corrected->gyro_rad_delt);
	Vector3_Add(&corrected->gyro_rad_delt, &samples[1].gyro_rad_delt,
			&corrected->gyro_rad_delt);
	Vector3_Add(&corrected->gyro_rad_delt, &samples[2].gyro_rad_delt,
			&corrected->gyro_rad_delt);
	Vector3_Add(&corrected->gyro_rad_delt, &samples[3].gyro_rad_delt,
			&corrected->gyro_rad_delt);
}

static void Sculling_Compensate_4Sample(const ImuMeasurement_t samples[4],
		ImuMeasurement_t *corrected) {
	Vector3_t theta_term;
	Vector3_t velocity_term;
	Vector3_t scull;

	Vector3_t temp;

	Vector3_Scale(&samples[0]->gyro_rad_delt, SCULL_K1_THETA, &theta_term);
	Vector3_Scale(&samples[1]->gyro_rad_delt, SCULL_K2_THETA, &temp);
	Vector3_Add(&theta_term, &temp, &theta_term);
	Vector3_Scale(&samples[2]->gyro_rad_delt, SCULL_K3_THETA, &temp);
	Vector3_Add(&theta_term, &temp, &theta_term);

	Vector3_Scale(&samples[0]->accel_m_s_delt, SCULL_K1_V, &velocity_term);
	Vector3_Scale(&samples[1]->accel_m_s_delt, SCULL_K2_V, &temp);
	Vector3_Add(&velocity_term, &temp, &velocity_term);
	Vector3_Scale(&samples[2]->accel_m_s_delt, SCULL_K3_V, &temp);
	Vector3_Add(&velocity_term, &temp, &velocity_term);

	Vector3_Cross(&theta_term, samples[3]->accel_m_s_delt, &scull);
	Vector3_Cross(&velocity_term, samples[3]->gyro_rad_delt, &temp);

	Vector3_Add(&scull, &temp, &corrected->accel_m_s_delt);
}
