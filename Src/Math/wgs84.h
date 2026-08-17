/*
 * wgs84.h
 *
 *  Created on: 13-Aug-2026
 *      Author: abhimanyu
 */

#ifndef WGS84_H_
#define WGS84_H_

#include "math_types.h"

/*
 * WGS-84 reference ellipsoid constants.
 */
#define WGS84_SEMI_MAJOR_AXIS_M       (6378137.0)
#define WGS84_FLATTENING              (1.0 / 298.257223563)
#define WGS84_EARTH_ROTATION_RAD_S    (7.2921151467e-5)
#define WGS84_STANDARD_GRAVITY_M_S2   (9.80665)

/**
 * @brief WGS-84 ellipsoid parameters.
 */
typedef struct {
	float64_t semi_major_axis_m;
	float64_t flattening;
	float64_t eccentricity_squared;
} Wgs84Ellipsoid_t;

/**
 * @brief Earth radii at a geodetic latitude.
 */
typedef struct {
	float64_t prime_vertical_radius_m;
	float64_t meridian_radius_m;
} Wgs84Radii_t;

/**
 * @brief Earth gravity model result.
 *
 * Gravity is expressed as a positive downward
 * acceleration magnitude.
 */
typedef struct {
	float64_t gravity_m_s2;
} Wgs84Gravity_t;

/**
 * @brief Earth rotation and transport rates.
 *
 * All angular rates are expressed in NED.
 */
typedef struct {
	Vector3_t earth_rate_ned_rad_s;
	Vector3_t transport_rate_ned_rad_s;
} Wgs84AngularRates_t;

/**
 * @brief Initialize a WGS-84 ellipsoid.
 *
 * @param ellipsoid Output ellipsoid.
 */
void Wgs84_Init(Wgs84Ellipsoid_t *ellipsoid);

/**
 * @brief Calculate WGS-84 radii of curvature.
 *
 * @param ellipsoid WGS-84 ellipsoid.
 * @param latitude_rad Geodetic latitude [rad].
 * @param radii Output radii.
 *
 * @return true if calculation is valid.
 */
bool Wgs84_CalculateRadii(const Wgs84Ellipsoid_t *ellipsoid,
		float64_t latitude_rad, Wgs84Radii_t *radii);

/**
 * @brief Calculate normal gravity.
 *
 * @param ellipsoid WGS-84 ellipsoid.
 * @param latitude_rad Geodetic latitude [rad].
 * @param altitude_m Geodetic altitude [m].
 * @param gravity Output gravity.
 *
 * @return true if calculation is valid.
 */
bool Wgs84_CalculateGravity(const Wgs84Ellipsoid_t *ellipsoid,
		float64_t latitude_rad, float64_t altitude_m, Wgs84Gravity_t *gravity);

/**
 * @brief Calculate Earth rotation and transport rates.
 *
 * @param ellipsoid WGS-84 ellipsoid.
 * @param latitude_rad Geodetic latitude [rad].
 * @param altitude_m Geodetic altitude [m].
 * @param velocity_ned_m_s NED velocity [m/s].
 * @param rates Output angular rates.
 *
 * @return true if calculation is valid.
 */
bool Wgs84_CalculateAngularRates(const Wgs84Ellipsoid_t *ellipsoid,
		float64_t latitude_rad, float64_t altitude_m,
		const Vector3_t *velocity_ned_m_s, Wgs84AngularRates_t *rates);

/**
 * @brief Calculate geodetic position rate.
 *
 * @param ellipsoid WGS-84 ellipsoid.
 * @param latitude_rad Geodetic latitude [rad].
 * @param altitude_m Geodetic altitude [m].
 * @param velocity_ned_m_s NED velocity [m/s].
 * @param position_rate Output position rate.
 *
 * @return true if calculation is valid.
 */
bool Wgs84_CalculatePositionRate(const Wgs84Ellipsoid_t *ellipsoid,
		float64_t latitude_rad, float64_t altitude_m,
		const Vector3_t *velocity_ned_m_s,
		GeodeticPositionRate_t *position_rate);

#endif /* WGS84_H_ */
