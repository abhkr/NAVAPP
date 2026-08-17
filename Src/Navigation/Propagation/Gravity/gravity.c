/*
 * gravity.c
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#include "gravity.h"

#include <math.h>

#define GRAVITY_EQUATOR_M_S2       (9.7803253359)
#define GRAVITY_K                  (0.00193185265241)
#define EARTH_ECCENTRICITY_SQ      (0.00669437999014)

void Gravity_Init(NavigationGravity_t *gravity) {
	if (gravity != NULL) {
		gravity->gravity_m_s2 =
		GRAVITY_EQUATOR_M_S2;

		gravity->gravity_ned_m_s2.x = 0.0;
		gravity->gravity_ned_m_s2.y = 0.0;
		gravity->gravity_ned_m_s2.z =
		GRAVITY_EQUATOR_M_S2;
	}
}

void Gravity_Update(NavigationGravity_t *gravity,
		const NavigationPosition_t *position) {
	double sin_latitude;
	double sin_latitude_squared;
	double gravity_m_s2;

	if ((gravity != NULL) && (position != NULL)) {
		sin_latitude = sin(position->latitude_rad);

		sin_latitude_squared = sin_latitude * sin_latitude;

		gravity_m_s2 =
		GRAVITY_EQUATOR_M_S2 * (1.0 + (GRAVITY_K * sin_latitude_squared))
				/ sqrt(1.0 - (EARTH_ECCENTRICITY_SQ * sin_latitude_squared));

		gravity->gravity_m_s2 = gravity_m_s2;

		gravity->gravity_ned_m_s2.x = 0.0;

		gravity->gravity_ned_m_s2.y = 0.0;

		/*
		 * NED convention:
		 * positive Down.
		 */
		gravity->gravity_ned_m_s2.z = gravity_m_s2;
	}
}
