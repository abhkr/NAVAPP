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
#define WGS84_SEMI_MINOR_AXIS_M       (6356752.314245179)
#define WGS84_FLATTENING              (1.0 / 298.257223563)
#define WGS84_EARTH_ROTATION_RAD_S    (7.2921151467e-5)
#define WGS84_FIRST_ECCENTRICITY_SQ   (6.6943799901413165e-3)

#define WGS84_EQUATOR_GRAVITY_MPS2    (9.7803253359)
#define WGS84_POLAR_GRAVITY_MPS2      (9.8321849378)

#define WGS84_GRAVITY_K               (0.00193185265241)

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
	float64_t n_gravity;
	float64_t e_gravity;
	float64_t d_gravity;
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

typedef struct {
	double latitude_rad;
	double longitude_rad;
	double altitude_m;

} Lla_t;

MathStatus_t WGS84_LlaToEcef(const GeodeticPosition_t *lla, Vector3_t *ecef_m);

/**
 * @brief Calculate WGS-84 radii of curvature.
 *
 * @param ellipsoid WGS-84 ellipsoid.
 * @param latitude_rad Geodetic latitude [rad].
 * @param radii Output radii.
 *
 * @return true if calculation is valid.
 */
bool Wgs84_CalculateRadii(const float64_t latitude_rad, Wgs84Radii_t *radii);

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
bool Wgs84_CalculateGravity(float64_t latitude_rad, float64_t altitude_m,
		Wgs84Gravity_t *gravity);

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
bool Wgs84_CalculateAngularRates(float64_t latitude_rad, float64_t altitude_m,
		const NedVelocity_t *velocity_ned_m_s, Wgs84AngularRates_t *rates);

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
bool Wgs84_CalculatePositionRate(float64_t latitude_rad, float64_t altitude_m,
		const Vector3_t *velocity_ned_m_s,
		GeodeticPositionRate_t *position_rate);

#endif /* WGS84_H_ */
