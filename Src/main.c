/*
 * main.c
 *
 *  Created on: 07-Aug-2026
 *      Author: ngarm-ins
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "matrix.h"
#include "math_types.h"

//#include "utils.h"

int main(void) {
	Matrix3_t mat;
	uint8_t i, j;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			mat.m_data[i][j] = i * 3 + j;
		}
	}

	printf("%lf\t%lf\t%lf\n", mat.m00, mat.m01, mat.m02);
	printf("%lf\t%lf\t%lf\n", mat.m10, mat.m11, mat.m12);
	printf("%lf\t%lf\t%lf\n", mat.m20, mat.m21, mat.m22);

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			printf("%lf\t", mat.m_data[i][j]);
		}
		printf("\n");
	}

	// Initial Conditions
//	EulerAngles e = { 1.0 * CDR, 2.0 * CDR, 3.0 * CDR };
//	Quaternion q_nb = { 1.0, 0.0, 0.0, 0.0 }; // Aligned with NED frame
//	//q_nb = EulerToQuaternion()
//	double lat = 0.52359877559; // ~30 degrees latitude in radians
//	double h = 100.0;           // 100 meters altitude
//	Vector3 v_ned = { 100.0, 50.0, 0.0 }; // Velocity: 100 m/s North, 50 m/s East
//
//	// Gyro Measurement (100 deg/s around Body Z-axis converted to rad/s)
//	double deg2rad = 3.14159265358979323846 / 180.0;
//	Vector3 w_ib_b = { 0.0, 0.0, 100.0 * deg2rad };
//	double dt = 0.1; // 100 ms execution cycle
//
//	// Perform Update
//	q_nb = ins_quaternion_update(q_nb, w_ib_b, v_ned, lat, h, dt);
//
//	// Display Output
//	printf("Updated Quaternion (q_nb):\n");
//	printf("q0 = %.9f\n", q_nb.q0);
//	printf("q1 = %.9f\n", q_nb.q1);
//	printf("q2 = %.9f\n", q_nb.q2);
//	printf("q3 = %.9f\n", q_nb.q3);

	return 0;
}
