/*
 * main.c
 *
 *  Created on: 07-Aug-2026
 *      Author: ngarm-ins
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "dcm.h"
#include "matrix.h"
#include "math_constants.h"
#include "transform.h"
#include "quaternion.h"
#include "math_types.h"

int main(void) {
	uint8_t i, j;
	EulerAngles_t euler;
	EulerAngles_t euler2;
	Matrix3_t dcm;
//	Quaternion_t quat;

	double ang[3] = { 10.0, 89.0, -10.0 };

	euler.yaw_rad = ang[0] * MATH_DEG_TO_RAD;
	euler.pitch_rad = ang[1] * MATH_DEG_TO_RAD;
	euler.roll_rad = ang[2] * MATH_DEG_TO_RAD;

//	for (i = 0; i < 3; i++) {
//		for (j = 0; j < 3; j++) {
//			printf("%lf\t", dcm.m_data[i][j]);
//		}
//		printf("\n");
//	}

	Transform_EulerToDcm(&euler, &dcm);
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			printf("%lf\t", dcm.m_data[i][j]);
		}
		printf("\n");
	}

	Transform_DcmToEuler(&dcm, &euler2);

//	Euler_FromDcm_stp(&dcm, &euler2);

	printf("SI - %lf\n", euler2.yaw_rad * MATH_RAD_TO_DEG);
	printf("THETA - %lf\n", euler2.pitch_rad * MATH_RAD_TO_DEG);
	printf("PHI - %lf\n", euler2.roll_rad * MATH_RAD_TO_DEG);

//	Transform_DcmToQuaternion(&dcm, &quat);
//	printf("q = %lf,%lf,%lf,%lf\n", quat.w, quat.x, quat.y, quat.z);
//
//	Transform_EulerToQuaternion(&euler, &quat);
//	printf("q = %lf,%lf,%lf,%lf\n", quat.w, quat.x, quat.y, quat.z);
//
//	Transform_QuaternionToEuler(&quat, &euler2);
//
//	printf("SI - %lf\n", euler2.yaw_rad * MATH_RAD_TO_DEG);
//	printf("THETA - %lf\n", euler2.pitch_rad * MATH_RAD_TO_DEG);
//	printf("PHI - %lf\n", euler2.roll_rad * MATH_RAD_TO_DEG);

	return 0;
}
