/*
 * wgs84.c
 *
 *  Created on: 13-Aug-2026
 *      Author: abhimanyu
 */

#include "wgs84.h"

#include <math.h>
#include <stddef.h>

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

static float64_t Wgs84_CalculateEccentricitySquared(
		const Wgs84Ellipsoid_t *ellipsoid);

static float64_t Wgs84_CalculatePrimeVerticalRadius(
		const Wgs84Ellipsoid_t *ellipsoid, float64_t latitude_rad);

static float64_t Wgs84_CalculateMeridianRadius(
		const Wgs84Ellipsoid_t *ellipsoid, float64_t latitude_rad);

static float64_t Wgs84_CalculateNormalGravity(float64_t latitude_rad);

void Wgs84_Init(Wgs84Ellipsoid_t *ellipsoid) {
	if (ellipsoid != NULL) {
		ellipsoid->semi_major_axis_m =
		WGS84_SEMI_MAJOR_AXIS_M;

		ellipsoid->flattening =
		WGS84_FLATTENING;

		ellipsoid->eccentricity_squared = Wgs84_CalculateEccentricitySquared(
				ellipsoid);
	}
}

bool Wgs84_CalculateRadii(const Wgs84Ellipsoid_t *ellipsoid,
		float64_t latitude_rad, Wgs84Radii_t *radii) {
	bool status;

	status = false;

	if ((ellipsoid != NULL) && (radii != NULL)
			&& Wgs84_IsValidLatitude(latitude_rad)) {
		radii->prime_vertical_radius_m = Wgs84_CalculatePrimeVerticalRadius(
				ellipsoid, latitude_rad);

		radii->meridian_radius_m = Wgs84_CalculateMeridianRadius(ellipsoid,
				latitude_rad);

		status = true;
	}

	return status;
}

bool Wgs84_CalculateGravity(const Wgs84Ellipsoid_t *ellipsoid,
		float64_t latitude_rad, float64_t altitude_m, Wgs84Gravity_t *gravity) {
	float64_t surface_gravity;
	float64_t altitude_factor;
	float64_t radius_ratio;

	bool status;

	status = false;

	if ((ellipsoid != NULL) && (gravity != NULL)
			&& Wgs84_IsValidLatitude(latitude_rad)
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
		radius_ratio = altitude_m /
		WGS84_EQUATORIAL_RADIUS_M;

		altitude_factor = 1.0 - (2.0 * radius_ratio);

		gravity->gravity_m_s2 = surface_gravity * altitude_factor;

		status = true;
	}

	return status;
}

bool Wgs84_CalculateAngularRates(const Wgs84Ellipsoid_t *ellipsoid,
		float64_t latitude_rad, float64_t altitude_m,
		const Vector3_t *velocity_ned_m_s, Wgs84AngularRates_t *rates) {
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

	if ((ellipsoid != NULL) && (velocity_ned_m_s != NULL) && (rates != NULL)
			&& Wgs84_IsValidLatitude(latitude_rad) &&
			Wgs84_CalculateRadii(
					ellipsoid,
					latitude_rad,
					&radii) == true) {
		sin_latitude = sin(latitude_rad);

		cos_latitude = cos(latitude_rad);

		velocity_north = velocity_ned_m_s->x;

		velocity_east = velocity_ned_m_s->y;

		earth_rate =
		WGS84_EARTH_ROTATION_RAD_S;

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

bool Wgs84_CalculatePositionRate(const Wgs84Ellipsoid_t *ellipsoid,
		float64_t latitude_rad, float64_t altitude_m,
		const Vector3_t *velocity_ned_m_s,
		GeodeticPositionRate_t *position_rate) {
	Wgs84Radii_t radii;

	float64_t cos_latitude;

	bool status;

	status = false;

	if ((ellipsoid != NULL) && (velocity_ned_m_s != NULL)
			&& (position_rate != NULL) && Wgs84_IsValidLatitude(latitude_rad) &&
			Wgs84_CalculateRadii(
					ellipsoid,
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

static float64_t Wgs84_CalculateEccentricitySquared(
		const Wgs84Ellipsoid_t *ellipsoid) {
	float64_t eccentricity_squared;

	eccentricity_squared = ellipsoid->flattening
			* (2.0 - ellipsoid->flattening);

	return eccentricity_squared;
}

static float64_t Wgs84_CalculatePrimeVerticalRadius(
		const Wgs84Ellipsoid_t *ellipsoid, float64_t latitude_rad) {
	float64_t sin_latitude;
	float64_t denominator;

	sin_latitude = sin(latitude_rad);

	denominator = sqrt(
			1.0
					- (ellipsoid->eccentricity_squared * sin_latitude
							* sin_latitude));

	if (denominator <
	WGS84_MIN_DENOMINATOR) {
		return ellipsoid->semi_major_axis_m;
	}

	return ellipsoid->semi_major_axis_m / denominator;
}

static float64_t Wgs84_CalculateMeridianRadius(
		const Wgs84Ellipsoid_t *ellipsoid, float64_t latitude_rad) {
	float64_t sin_latitude;
	float64_t denominator;

	sin_latitude = sin(latitude_rad);

	denominator = sqrt(
			1.0
					- (ellipsoid->eccentricity_squared * sin_latitude
							* sin_latitude));

	if (denominator <
	WGS84_MIN_DENOMINATOR) {
		return ellipsoid->semi_major_axis_m;
	}

	return (ellipsoid->semi_major_axis_m
			* (1.0 - ellipsoid->eccentricity_squared))
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
	numerator = 1.0 + (0.00193185265241 * sin_squared);

	denominator = sqrt(1.0 - (WGS84_ECCENTRICITY_SQUARED * sin_squared));

	/*
	 * Equatorial normal gravity.
	 */
	return 9.7803253359 * numerator / denominator;
}
