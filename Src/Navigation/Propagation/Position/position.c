/*
 * position.c
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#include "position.h"

#include <math.h>

#define EARTH_SEMI_MAJOR_AXIS_M    (6378137.0)
#define EARTH_ECCENTRICITY_SQ      (0.00669437999014)

static void Position_CalculateRadii(double latitude_rad,
		double *radius_meridian_m, double *radius_transverse_m);

void Position_Init(NavigationPosition_t *position) {
	if (position != NULL) {
		position->latitude_rad = 0.0;
		position->longitude_rad = 0.0;
		position->altitude_m = 0.0;
	}
}

void Position_Propagate(NavigationPosition_t *position,
		const NavigationVelocity_t *velocity, double dt_s) {
	double radius_meridian_m;
	double radius_transverse_m;
	double latitude_rad;

	if ((position != NULL) && (velocity != NULL)) {
		latitude_rad = position->latitude_rad;

		Position_CalculateRadii(latitude_rad, &radius_meridian_m,
				&radius_transverse_m);

		position->latitude_rad += (velocity->north_m_s
				/ (radius_meridian_m + position->altitude_m)) * dt_s;

		position->longitude_rad += (velocity->east_m_s
				/ ((radius_transverse_m + position->altitude_m)
						* cos(latitude_rad))) * dt_s;

		position->altitude_m -= velocity->down_m_s * dt_s;
	}
}

static void Position_CalculateRadii(double latitude_rad,
		double *radius_meridian_m, double *radius_transverse_m) {
	double sin_latitude;
	double denominator;

	sin_latitude = sin(latitude_rad);

	denominator = 1.0 - (EARTH_ECCENTRICITY_SQ * sin_latitude * sin_latitude);

	*radius_transverse_m =
	EARTH_SEMI_MAJOR_AXIS_M / sqrt(denominator);

	*radius_meridian_m =
	EARTH_SEMI_MAJOR_AXIS_M * (1.0 -
	EARTH_ECCENTRICITY_SQ) / pow(denominator, 1.5);
}
