/*
 * wgs84.c
 *
 *  Created on: 13-Aug-2026
 *      Author: abhimanyu
 */

#include "wgs84.h"

#include <stdio.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "vector3.h"

#define WGS84_EQUATORIAL_RADIUS_M (6378137.0)

#define WGS84_POLAR_RADIUS_M (6356752.314245179)

#define WGS84_ECCENTRICITY_SQUARED (0.0066943799901413165)

#define WGS84_SECOND_ECCENTRICITY_SQUARED (0.006739496742276434)

#define WGS84_J2 (1.08262668e-3)

#define WGS84_MIN_COS_LATITUDE (1.0e-12)

#define WGS84_MIN_DENOMINATOR (1.0e-12)

#define WGS84_GRAVITY_ALTITUDE_LIMIT_M (100000.0)

static bool Wgs84_IsValidLatitude(float64_t latitude_rad);

static float64_t Wgs84_CalculatePrimeVerticalRadius(float64_t latitude_rad);

static float64_t Wgs84_CalculateMeridianRadius(float64_t latitude_rad);

static float64_t Wgs84_CalculateNormalGravity(float64_t latitude_rad);

MathStatus_t WGS84_LlaToEcef(const GeodeticPosition_t *lla, Vector3_t *ecef_m) {
	MathStatus_t status;

	double eccentricity_squared;
	double sin_latitude;
	double cos_latitude;
	double sin_longitude;
	double cos_longitude;
	double denominator;
	double prime_vertical_radius_m;

	status = MATH_STATUS_NULL_POINTER;

	if ((lla != NULL) && (ecef_m != NULL)) {
		eccentricity_squared =
		WGS84_FLATTENING * (2.0 - WGS84_FLATTENING);

		sin_latitude = sin(lla->latitude_rad);

		cos_latitude = cos(lla->latitude_rad);

		sin_longitude = sin(lla->longitude_rad);

		cos_longitude = cos(lla->longitude_rad);

		denominator = sqrt(
				1.0 - (eccentricity_squared * sin_latitude * sin_latitude));

		prime_vertical_radius_m =
		WGS84_SEMI_MAJOR_AXIS_M / denominator;

		ecef_m->x = (prime_vertical_radius_m + lla->altitude_m) * cos_latitude
				* cos_longitude;

		ecef_m->y = (prime_vertical_radius_m + lla->altitude_m) * cos_latitude
				* sin_longitude;

		ecef_m->z = ((prime_vertical_radius_m * (1.0 - eccentricity_squared))
				+ lla->altitude_m) * sin_latitude;

		status = MATH_STATUS_OK;
	}

	return status;
}

bool Wgs84_CalculateRadii(float64_t latitude_rad, Wgs84Radii_t *radii) {
	bool status;

	status = false;

	if ((radii != NULL) && Wgs84_IsValidLatitude(latitude_rad)) {
		radii->prime_vertical_radius_m = Wgs84_CalculatePrimeVerticalRadius(
				latitude_rad);

		radii->meridian_radius_m = Wgs84_CalculateMeridianRadius(latitude_rad);

		status = true;
	}

	return status;
}

bool Wgs84_CalculateGravity(float64_t latitude_rad, float64_t altitude_m,
		Wgs84Gravity_t *gravity) {
	float64_t surface_gravity;
	float64_t altitude_factor;
	float64_t radius_ratio;

	bool status;

	status = false;

	if ((gravity != NULL) && Wgs84_IsValidLatitude(latitude_rad)
			&& (altitude_m >= -WGS84_GRAVITY_ALTITUDE_LIMIT_M) && (altitude_m <=
			WGS84_GRAVITY_ALTITUDE_LIMIT_M)) {
		surface_gravity = Wgs84_CalculateNormalGravity(latitude_rad);

		/*
		 * First-order altitude correction.
		 *
		 * This is appropriate for the INS operating
		 * altitude range. A higher-order gravity model
		 * can be added later if required.
		 */
		radius_ratio = altitude_m / WGS84_EQUATORIAL_RADIUS_M;

		altitude_factor = 1.0 - (2.0 * radius_ratio)
				+ (3.0 * radius_ratio * radius_ratio);

		gravity->gravity_m_s2 = surface_gravity * altitude_factor;

		gravity->n_gravity = 0.0;
		gravity->e_gravity = 0.0;
		gravity->d_gravity = gravity->gravity_m_s2;

		status = true;
	}

	return status;
}

bool Wgs84_CalculateAngularRates(float64_t latitude_rad, float64_t altitude_m,
		const NedVelocity_t *velocity_ned_m_s, Wgs84AngularRates_t *rates) {
	Wgs84Radii_t radii;

	float64_t sin_latitude;
	float64_t cos_latitude;

	float64_t velocity_north;
	float64_t velocity_east;

	float64_t earth_rate;

	float64_t transport_north;
	float64_t transport_east;
	float64_t transport_down;

	bool status;

	status = false;

	if ((velocity_ned_m_s != NULL) && (rates != NULL)
			&& Wgs84_IsValidLatitude(latitude_rad) &&
			Wgs84_CalculateRadii(latitude_rad,&radii) == true) {
		sin_latitude = sin(latitude_rad);

		cos_latitude = cos(latitude_rad);

		velocity_north = velocity_ned_m_s->north_m_s;

		velocity_east = velocity_ned_m_s->east_m_s;

		earth_rate = WGS84_EARTH_ROTATION_RAD_S;

		/*
		 * Earth rotation rate expressed in NED.
		 *
		 * omega_ie^n =
		 * [
		 *   omega_ie cos(L)
		 *   0
		 *  -omega_ie sin(L)
		 * ]
		 */
		rates->earth_rate_ned_rad_s.x = earth_rate * cos_latitude;

		rates->earth_rate_ned_rad_s.y = 0.0;

		rates->earth_rate_ned_rad_s.z = -earth_rate * sin_latitude;

		/*
		 * Transport rate:
		 *
		 * omega_en^n =
		 * [
		 *   Ve / (RN + h)
		 *  -Vn / (RM + h)
		 *  -Ve tan(L) / (RN + h)
		 * ]
		 */
		transport_north = velocity_east
				/ (radii.prime_vertical_radius_m + altitude_m);

		transport_east = -velocity_north
				/ (radii.meridian_radius_m + altitude_m);

		if (fabs(cos_latitude) >
		WGS84_MIN_COS_LATITUDE) {
			transport_down = -velocity_east * sin_latitude
					/ ((radii.prime_vertical_radius_m + altitude_m)
							* cos_latitude);
		} else {
			transport_down = 0.0;
		}

		rates->transport_rate_ned_rad_s.x = transport_north;

		rates->transport_rate_ned_rad_s.y = transport_east;

		rates->transport_rate_ned_rad_s.z = transport_down;

		(void) Wgs84_IsValidLatitude(latitude_rad);

		status = true;
	}

	return status;
}

bool Wgs84_CalculatePositionRate(float64_t latitude_rad, float64_t altitude_m,
		const Vector3_t *velocity_ned_m_s,
		GeodeticPositionRate_t *position_rate) {
	Wgs84Radii_t radii;

	float64_t cos_latitude;

	bool status;

	status = false;

	if ((velocity_ned_m_s != NULL) && (position_rate != NULL)
			&& Wgs84_IsValidLatitude(latitude_rad) &&
			Wgs84_CalculateRadii(
					latitude_rad,
					&radii) == true) {
		cos_latitude = cos(latitude_rad);

		position_rate->latitude_rate_rad_s = velocity_ned_m_s->x
				/ (radii.meridian_radius_m + altitude_m);

		if (fabs(cos_latitude) >
		WGS84_MIN_COS_LATITUDE) {
			position_rate->longitude_rate_rad_s = velocity_ned_m_s->y
					/ ((radii.prime_vertical_radius_m + altitude_m)
							* cos_latitude);
		} else {
			position_rate->longitude_rate_rad_s = 0.0;
		}

		position_rate->altitude_rate_m_s = -velocity_ned_m_s->z;

		status = true;
	}

	return status;
}

static bool Wgs84_IsValidLatitude(float64_t latitude_rad) {
	return ((latitude_rad >= -1.5707963267948966)
			&& (latitude_rad <= 1.5707963267948966));
}

static float64_t Wgs84_CalculatePrimeVerticalRadius(float64_t latitude_rad) {
	float64_t sin_latitude;
	float64_t denominator;

	sin_latitude = sin(latitude_rad);

	denominator = sqrt(
			1.0 - (WGS84_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude));

	if (denominator < WGS84_MIN_DENOMINATOR) {
		return WGS84_SEMI_MAJOR_AXIS_M;
	}

	return WGS84_SEMI_MAJOR_AXIS_M / denominator;
}

static float64_t Wgs84_CalculateMeridianRadius(float64_t latitude_rad) {
	float64_t sin_latitude;
	float64_t denominator;

	sin_latitude = sin(latitude_rad);

	denominator = sqrt(
			1.0 - (WGS84_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude));

	if (denominator <
	WGS84_MIN_DENOMINATOR) {
		return WGS84_SEMI_MAJOR_AXIS_M;
	}

	return (WGS84_SEMI_MAJOR_AXIS_M * (1.0 - WGS84_ECCENTRICITY_SQUARED))
			/ (denominator * denominator * denominator);
}

static float64_t Wgs84_CalculateNormalGravity(float64_t latitude_rad) {
	float64_t sin_latitude;
	float64_t sin_squared;
	float64_t numerator;
	float64_t denominator;

	sin_latitude = sin(latitude_rad);

	sin_squared = sin_latitude * sin_latitude;

	/*
	 * Somigliana normal gravity formula.
	 *
	 * gamma =
	 * gamma_e *
	 * (1 + k sin²(L)) /
	 * sqrt(1 - e² sin²(L))
	 */
	numerator = 1.0 + (WGS84_GRAVITY_K * sin_squared);

	denominator = sqrt(1.0 - (WGS84_ECCENTRICITY_SQUARED * sin_squared));

	/*
	 * Equatorial normal gravity.
	 */
	return ((WGS84_EQUATOR_GRAVITY_MPS2 * numerator) / denominator);
}
