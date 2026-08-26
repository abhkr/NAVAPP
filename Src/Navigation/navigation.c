#include "navigation.h"

#include <math.h>
#include <stddef.h>

#include "dcm.h"
#include "quaternion.h"
#include "transform.h"
#include "matrix.h"
#include "math_types.h"

///* ==========================================================================
// * Constants
// * ========================================================================== */
//
//#define NAVIGATION_PI                 (3.14159265358979323846)
//
//#define NAVIGATION_DEG_TO_RAD		(NAVIGATION_PI / 180.0)
//
//#define NAVIGATION_RAD_TO_DEG     (180.0 / NAVIGATION_PI)
//
///*
// * WGS-84 parameters.
// */
//#define NAVIGATION_EARTH_SEMI_MAJOR_M (6378137.0)
//
//#define NAVIGATION_EARTH_FLATTENING   (1.0 / 298.257223563)
//
//#define NAVIGATION_EARTH_ROTATION_RAD_S (7.2921150e-5)
//
//#define NAVIGATION_EARTH_ECCENTRICITY_SQ (6.6943799901413165e-3)
//
/* ==========================================================================
 * Private function declarations
 * ========================================================================== */

static void Navigation_ResetImuBuffer(Navigation_t *navigation);
//
//static void Navigation_AverageImuSamples(Navigation_t *navigation,
//		ImuMeasurement_t *average);
//
//static void Navigation_PreprocessImu(Navigation_t *navigation,
//		const ImuMeasurement_t *imu);
//
//static void Navigation_UpdateGravity(Navigation_t *navigation);
//
//static void Navigation_UpdateEarthRate(Navigation_t *navigation);
//
//static void Navigation_UpdateTransportRate(Navigation_t *navigation);
//
//static void Navigation_UpdateAttitude(Navigation_t *navigation);
//
//static void Navigation_UpdateVelocity(Navigation_t *navigation);
//
//static void Navigation_UpdatePosition(Navigation_t *navigation);
//
//static void Navigation_UpdateRadii(Navigation_t *navigation);
//
/* ==========================================================================
 * Initialization
 * ========================================================================== */

void Navigation_Init_From_Mdl(const NavigationMdl_t *mdl_data,
		Navigation_t *navigation) {

	if (navigation != NULL) {
		Navigation_ResetImuBuffer(navigation);

		navigation->rcnt = 0U;

		/* Initialize INS Position */
		navigation->pure_solution.position.latitude_rad =
				mdl_data->position.latitude_rad;
		navigation->pure_solution.position.longitude_rad =
				mdl_data->position.longitude_rad;
		navigation->pure_solution.position.altitude_m =
				mdl_data->position.altitude_m;

		/* Initialize INS Velocity */
		navigation->pure_solution.velocity.north_m_s =
				mdl_data->velocity.north_m_s;
		navigation->pure_solution.velocity.east_m_s =
				mdl_data->velocity.east_m_s;
		navigation->pure_solution.velocity.down_m_s =
				mdl_data->velocity.down_m_s;

		/* Initialize Attitude */
		navigation->pure_solution.attitude.yaw_rad = mdl_data->attitude.yaw_rad;
		navigation->pure_solution.attitude.pitch_rad =
				mdl_data->attitude.pitch_rad;
		navigation->pure_solution.attitude.roll_rad =
				mdl_data->attitude.roll_rad;

		/* Initialize Quaternion & DCM - Ned 2 Body */
		Transform_EulerToQuaternion(&mdl_data->attitude,
				&navigation->pure_solution.quaternion);
		Transform_EulerToDcm(&mdl_data->attitude, &navigation->dcm_ned_to_body);

		/* Initialize Earth Radii */
		Wgs84_CalculateRadii(navigation->pure_solution.position.latitude_rad,
				&navigation->radius);

		/* Initialize earth rotation */
		Wgs84_CalculateAngularRates(
				navigation->pure_solution.position.latitude_rad,
				navigation->pure_solution.position.altitude_m,
				&navigation->pure_solution.velocity, &navigation->rates);

	}
}

///* ==========================================================================
// * IMU processing
// * ========================================================================== */
//
//void Navigation_ImuUpdate(Navigation_t *navigation, const ImuMeasurement_t *imu) {
//	if ((navigation != NULL) && (imu != NULL)
//			&& (navigation->initialized == true)) {
//		if (navigation->imu_sample_count <
//		NAVIGATION_IMU_SAMPLE_COUNT) {
//			navigation->imu_samples[navigation->imu_sample_count] = *imu;
//
//			navigation->imu_sample_count++;
//		}
//
//		/*
//		 * Four samples correspond to 10 ms.
//		 */
//		if (navigation->imu_sample_count >=
//		NAVIGATION_IMU_SAMPLE_COUNT) {
//			ImuMeasurement_t average_imu;
//
//			Navigation_AverageImuSamples(navigation, &average_imu);
//
//			Navigation_PreprocessImu(navigation, &average_imu);
//
//			Navigation_Update(navigation);
//
//			Navigation_ResetImuBuffer(navigation);
//		}
//	}
//}
//
///* ==========================================================================
// * Complete INS mechanization
// * ========================================================================== */
//
//void Navigation_Update(Navigation_t *navigation) {
//	if ((navigation != NULL) && (navigation->initialized == true)) {
//		/*
//		 * Update Earth model first because gravity,
//		 * Earth rate and transport rate depend on position
//		 * and velocity.
//		 */
//		Navigation_UpdateRadii(navigation);
//
//		Navigation_UpdateGravity(navigation);
//
//		Navigation_UpdateEarthRate(navigation);
//
//		Navigation_UpdateTransportRate(navigation);
//
//		/*
//		 * INS mechanization.
//		 *
//		 * The attitude is updated first because the updated
//		 * attitude is required to transform specific force
//		 * from body frame to navigation frame.
//		 */
//		Navigation_UpdateAttitude(navigation);
//
//		Navigation_UpdateVelocity(navigation);
//
//		Navigation_UpdatePosition(navigation);
//
//		navigation->update_available = true;
//	}
//}
//
static void Navigation_Coning_Compensate(const ImuMeasurement_t samples[4],
		Navigation_t *navigation) {

	Vector3_t theta_sum;

	Vector3_t cross_12;
	Vector3_t cross_23;
	Vector3_t cross_34;

	Vector3_t cross_13;
	Vector3_t cross_24;

	Vector3_t cross_14;

	Vector3_t k1_term;
	Vector3_t k2_term;
	Vector3_t k3_term;

	Vector3_t cone_total;

	/* ---------------------------------------------------------
	 * 1. Simple sum of four Angular increments
	 * --------------------------------------------------------- */

	Vector3_Add(&samples[0].gyro_rad_delt, &samples[1].gyro_rad_delt,
			&theta_sum);
	Vector3_Add(&theta_sum, &samples[2].gyro_rad_delt, &theta_sum);
	Vector3_Add(&theta_sum, &samples[3].gyro_rad_delt, &theta_sum);

	/* ---------------------------------------------------------
	 * 2. Adjacent-sample cross products - K1 terms
	 *
	 * theta1 x theta2
	 * theta2 x theta3
	 * theta3 x theta4
	 * --------------------------------------------------------- */

	Vector3_Cross(&samples[0].gyro_rad_delt, &samples[1].gyro_rad_delt,
			&cross_12);
	Vector3_Cross(&samples[1].gyro_rad_delt, &samples[2].gyro_rad_delt,
			&cross_23);
	Vector3_Cross(&samples[2].gyro_rad_delt, &samples[3].gyro_rad_delt,
			&cross_34);

	Vector3_Add(&cross_12, &cross_23, &k1_term);
	Vector3_Add(&k1_term, &cross_34, &k1_term);
	Vector3_Scale(&k1_term, CONE_K1_COEFF, &k1_term);

	/* ---------------------------------------------------------
	 * 3. One-sample-separated products - K2 terms
	 *
	 * theta1 x theta3
	 * theta2 x theta4
	 * --------------------------------------------------------- */

	Vector3_Cross(&samples[0].gyro_rad_delt, &samples[2].gyro_rad_delt,
			&cross_13);
	Vector3_Cross(&samples[1].gyro_rad_delt, &samples[3].gyro_rad_delt,
			&cross_24);

	Vector3_Add(&cross_13, &cross_24, &k2_term);
	Vector3_Scale(&k2_term, CONE_K2_COEFF, &k2_term);

	/* ---------------------------------------------------------
	 * 4. First-to-fourth sample - K3 terms
	 *
	 * theta1 x theta4
	 * --------------------------------------------------------- */

	Vector3_Cross(&samples[0].gyro_rad_delt, &samples[3].gyro_rad_delt,
			&cross_14);

	Vector3_Scale(&cross_14, CONE_K3_COEFF, &k3_term);

	/* ---------------------------------------------------------
	 * 5. Total coning correction
	 * --------------------------------------------------------- */
	Vector3_Add(&k1_term, &k2_term, &cone_total);
	Vector3_Add(&cone_total, &k3_term, &cone_total);

	/* ---------------------------------------------------------
	 * 6. Total body-frame velocity increment
	 * --------------------------------------------------------- */

	Vector3_Add(&theta_sum, &cone_total,
			&navigation->imu_compensated.gyro_rad_delt);
}

void Navigation_Sculling_Compensate(const ImuMeasurement_t samples[4],
		Navigation_t *navigation) {

	Vector3_t sum_delta_v;
	Vector3_t scull_k1;
	Vector3_t scull_k2;
	Vector3_t scull_k3;
	Vector3_t scull_total;

	Vector3_t term;

	/* ---------------------------------------------------------
	 * 1. Simple sum of four velocity increments
	 * --------------------------------------------------------- */

	Vector3_Add(&samples[0].accel_m_s_delt, &samples[1].accel_m_s_delt,
			&sum_delta_v);
	Vector3_Add(&sum_delta_v, &samples[2].accel_m_s_delt, &sum_delta_v);
	Vector3_Add(&sum_delta_v, &samples[3].accel_m_s_delt, &sum_delta_v);

	/* ---------------------------------------------------------
	 * 2. k1 terms
	 *
	 * theta1 x dv2
	 * dv1    x theta2
	 * theta2 x dv3
	 * dv2    x theta3
	 * --------------------------------------------------------- */

	scull_k1.x = 0.0;
	scull_k1.y = 0.0;
	scull_k1.z = 0.0;

	Vector3_Cross(&samples[0].gyro_rad_delt, &samples[1].accel_m_s_delt, &term);
	Vector3_Add(&scull_k1, &term, &scull_k1);

	Vector3_Cross(&samples[0].accel_m_s_delt, &samples[1].gyro_rad_delt, &term);
	Vector3_Add(&scull_k1, &term, &scull_k1);

	Vector3_Cross(&samples[1].gyro_rad_delt, &samples[2].accel_m_s_delt, &term);
	Vector3_Add(&scull_k1, &term, &scull_k1);

	Vector3_Cross(&samples[1].accel_m_s_delt, &samples[2].gyro_rad_delt, &term);
	Vector3_Add(&scull_k1, &term, &scull_k1);

	Vector3_Scale(&scull_k1, SCULL_K1_COEFF, &scull_k1);

	/* ---------------------------------------------------------
	 * 3. k2 terms
	 *
	 * theta1 x dv3
	 * dv1    x theta3
	 * theta2 x dv4
	 * dv2    x theta4
	 * --------------------------------------------------------- */

	scull_k2.x = 0.0;
	scull_k2.y = 0.0;
	scull_k2.z = 0.0;

	Vector3_Cross(&samples[0].gyro_rad_delt, &samples[2].accel_m_s_delt, &term);
	Vector3_Add(&scull_k2, &term, &scull_k2);

	Vector3_Cross(&samples[0].accel_m_s_delt, &samples[2].gyro_rad_delt, &term);
	Vector3_Add(&scull_k2, &term, &scull_k2);

	Vector3_Cross(&samples[1].gyro_rad_delt, &samples[3].accel_m_s_delt, &term);
	Vector3_Add(&scull_k2, &term, &scull_k2);

	Vector3_Cross(&samples[1].accel_m_s_delt, &samples[3].gyro_rad_delt, &term);
	Vector3_Add(&scull_k2, &term, &scull_k2);

	Vector3_Scale(&scull_k2, SCULL_K2_COEFF, &scull_k2);

	/* ---------------------------------------------------------
	 * 4. k3 terms
	 *
	 * theta1 x dv4
	 * dv1    x theta4
	 * --------------------------------------------------------- */

	scull_k3.x = 0.0;
	scull_k3.y = 0.0;
	scull_k3.z = 0.0;

	Vector3_Cross(&samples[0].gyro_rad_delt, &samples[3].accel_m_s_delt, &term);
	Vector3_Add(&scull_k3, &term, &scull_k3);

	Vector3_Cross(&samples[0].accel_m_s_delt, &samples[3].gyro_rad_delt, &term);
	Vector3_Add(&scull_k3, &term, &scull_k3);

	Vector3_Scale(&scull_k3, SCULL_K3_COEFF, &scull_k3);

	/* ---------------------------------------------------------
	 * 5. Total sculling correction
	 * --------------------------------------------------------- */

	Vector3_Add(&scull_k1, &scull_k2, &scull_total);
	Vector3_Add(&scull_total, &scull_k3, &scull_total);

	/* ---------------------------------------------------------
	 * 6. Total body-frame velocity increment
	 * --------------------------------------------------------- */

	Vector3_Add(&sum_delta_v, &scull_total,
			&navigation->imu_compensated.accel_m_s_delt);

}

void Navigation_Apply_Coning_Sculling(Navigation_t *navigation) {

	Vector3_Zero(&navigation->imu_compensated.gyro_rad_delt);
	Vector3_Zero(&navigation->imu_compensated.accel_m_s_delt);

	/* Coning Compensation */
	Navigation_Coning_Compensate(navigation->imu_samples, navigation);

	/* Sculling Compensation */
	Navigation_Sculling_Compensate(navigation->imu_samples, navigation);

}

/* ==========================================================================
 * IMU sample averaging
 * ========================================================================== */

//static void Navigation_AverageImuSamples(Navigation_t *navigation,
//		ImuMeasurement_t *average) {
//	uint32_t index;
//	double gyro_x;
//	double gyro_y;
//	double gyro_z;
//
//	double accel_x;
//	double accel_y;
//	double accel_z;
//
//	gyro_x = 0.0;
//	gyro_y = 0.0;
//	gyro_z = 0.0;
//
//	accel_x = 0.0;
//	accel_y = 0.0;
//	accel_z = 0.0;
//
//	if ((navigation != NULL) && (average != NULL)) {
//		for (index = 0U; index < NAVIGATION_IMU_SAMPLE_COUNT; index++) {
//			gyro_x += navigation->imu_samples[index].gyro_rad_delt.x;
//
//			gyro_y += navigation->imu_samples[index].gyro_rad_delt.y;
//
//			gyro_z += navigation->imu_samples[index].gyro_rad_delt.z;
//
//			accel_x += navigation->imu_samples[index].accel_m_s_delt.x;
//
//			accel_y += navigation->imu_samples[index].accel_m_s_delt.y;
//
//			accel_z += navigation->imu_samples[index].accel_m_s_delt.z;
//		}
//		average->gyro.x_rad_s = gyro_x / (double) NAVIGATION_IMU_SAMPLE_COUNT;
//
//		average->gyro.y_rad_s = gyro_y / (double) NAVIGATION_IMU_SAMPLE_COUNT;
//
//		average->gyro.z_rad_s = gyro_z / (double) NAVIGATION_IMU_SAMPLE_COUNT;
//
//		average->accelerometer.x_m_s2 = accel_x
//				/ (double) NAVIGATION_IMU_SAMPLE_COUNT;
//
//		average->accelerometer.y_m_s2 = accel_y
//				/ (double) NAVIGATION_IMU_SAMPLE_COUNT;
//
//		average->accelerometer.z_m_s2 = accel_z
//				/ (double) NAVIGATION_IMU_SAMPLE_COUNT;
//}
///* ==========================================================================
// * IMU preprocessing
// * ========================================================================== */
//
//static void Navigation_PreprocessImu(Navigation_t *navigation,
//		const ImuMeasurement_t *imu) {
//	if ((navigation != NULL) && (imu != NULL)) {
//		/*
//		 * At this stage the IMU data is already averaged.
//		 *
//		 * Sensor bias compensation, scale-factor correction,
//		 * misalignment correction and coning/sculling corrections
//		 * can be added here.
//		 *
//		 * For now:
//		 *
//		 * delta_angle = gyro * dt
//		 * delta_velocity = accelerometer * dt
//		 */
//
//		navigation->body_delta_angle.x_rad = imu->gyro_rad_delt.x *
//		NAVIGATION_UPDATE_PERIOD_S;
//
//		navigation->body_delta_angle.y_rad = imu->gyro_rad_delt.y *
//		NAVIGATION_UPDATE_PERIOD_S;
//
//		navigation->body_delta_angle.z_rad = imu->gyro_rad_delt.z *
//		NAVIGATION_UPDATE_PERIOD_S;
//
//		navigation->body_delta_velocity.x_m_s = imu->accel_m_s_delt.x *
//		NAVIGATION_UPDATE_PERIOD_S;
//
//		navigation->body_delta_velocity.y_m_s = imu->accel_m_s_delt.y *
//		NAVIGATION_UPDATE_PERIOD_S;
//
//		navigation->body_delta_velocity.z_m_s = imu->accel_m_s_delt.z *
//		NAVIGATION_UPDATE_PERIOD_S;
//	}
//}
//
///* ==========================================================================
// * Earth model
// * ========================================================================== */
//
//static void Navigation_UpdateRadii(Navigation_t *navigation) {
//	double sin_lat;
//	double denominator;
//
//	double latitude;
//
//	if (navigation != NULL) {
//		latitude = navigation->position.latitude_rad;
//
//		sin_lat = sin(latitude);
//
//		denominator = sqrt(
//				1.0 - (NAVIGATION_EARTH_ECCENTRICITY_SQ * sin_lat * sin_lat));
//
//		navigation->radii.transverse_m =
//		NAVIGATION_EARTH_SEMI_MAJOR_M / denominator;
//
//		navigation->radii.meridian_m = (NAVIGATION_EARTH_SEMI_MAJOR_M * (1.0 -
//		NAVIGATION_EARTH_ECCENTRICITY_SQ))
//				/ (denominator * denominator * denominator);
//	}
//}
//
///* ==========================================================================
// * Gravity
// * ========================================================================== */
//
//static void Navigation_UpdateGravity(Navigation_t *navigation) {
//	double latitude;
//	double altitude;
//
//	double sin_lat;
//	double sin_lat_sq;
//
//	double gravity_magnitude;
//
//	if (navigation != NULL) {
//		latitude = navigation->position.latitude_rad;
//
//		altitude = navigation->position.altitude_m;
//
//		sin_lat = sin(latitude);
//
//		sin_lat_sq = sin_lat * sin_lat;
//
//		/*
//		 * Normal gravity approximation.
//		 */
//		gravity_magnitude = 9.7803253359
//				* (1.0 + (0.00193185265241 * sin_lat_sq))
//				/ sqrt(1.0 - (NAVIGATION_EARTH_ECCENTRICITY_SQ * sin_lat_sq));
//
//		/*
//		 * Simple altitude correction.
//		 */
//		gravity_magnitude *= 1.0 - ((2.0 * altitude) /
//		NAVIGATION_EARTH_SEMI_MAJOR_M);
//
//		navigation->gravity.north_m_s2 = 0.0;
//
//		navigation->gravity.east_m_s2 = 0.0;
//
//		/*
//		 * NED convention:
//		 * Down is positive.
//		 */
//		navigation->gravity.down_m_s2 = gravity_magnitude;
//	}
//}
//
///* ==========================================================================
// * Earth rotation rate
// * ========================================================================== */
//
//static void Navigation_UpdateEarthRate(Navigation_t *navigation) {
//	double latitude;
//
//	if (navigation != NULL) {
//		latitude = navigation->position.latitude_rad;
//
//		navigation->earth_rate.north_rad_s =
//		NAVIGATION_EARTH_ROTATION_RAD_S * cos(latitude);
//
//		navigation->earth_rate.east_rad_s = 0.0;
//
//		navigation->earth_rate.down_rad_s = -NAVIGATION_EARTH_ROTATION_RAD_S
//				* sin(latitude);
//	}
//}
//
///* ==========================================================================
// * Transport rate
// * ========================================================================== */
//
//static void Navigation_UpdateTransportRate(Navigation_t *navigation) {
//	double latitude;
//	double altitude;
//
//	double velocity_north;
//	double velocity_east;
//
//	double radius_meridian;
//	double radius_transverse;
//
//	double cos_latitude;
//
//	if (navigation != NULL) {
//		latitude = navigation->position.latitude_rad;
//
//		altitude = navigation->position.altitude_m;
//
//		velocity_north = navigation->velocity.north_m_s;
//
//		velocity_east = navigation->velocity.east_m_s;
//
//		radius_meridian = navigation->radii.meridian_m;
//
//		radius_transverse = navigation->radii.transverse_m;
//
//		cos_latitude = cos(latitude);
//
//		navigation->transport_rate.north_rad_s = velocity_east
//				/ (radius_transverse + altitude);
//
//		navigation->transport_rate.east_rad_s = -velocity_north
//				/ (radius_meridian + altitude);
//
//		navigation->transport_rate.down_rad_s = -(velocity_east * tan(latitude))
//				/ (radius_transverse + altitude);
//
//		/*
//		 * Avoid numerical problems at the poles.
//		 */
//		if (fabs(cos_latitude) < 1.0e-8) {
//			navigation->transport_rate.north_rad_s = 0.0;
//			navigation->transport_rate.down_rad_s = 0.0;
//		}
//	}
//}
//
/* ==========================================================================
 * Attitude update
 *
 * Quaternion represents rotation from NED to body frame.
 *
 * q_NED_to_BODY
 *
 * With frame-rotation convention, the incremental body-frame
 * rotation is post-multiplied:
 *
 * q_new = q_old * dq
 * ========================================================================== */

void Navigation_UpdateAttitude(Navigation_t *navigation) {
	float64_t p;
	float64_t q;
	float64_t r;

	float64_t scale_term;
	float64_t theta_mag_square;

	Quaternion_t delta_quaternion_b;
	Quaternion_t delta_quaternion_n;
	Quaternion_t updated_quaternion;

	if (navigation != NULL) {
		/*
		 * Body angular increment.
		 */
		p = navigation->imu_compensated.gyro_rad_delt.x;

		q = navigation->imu_compensated.gyro_rad_delt.y;

		r = navigation->imu_compensated.gyro_rad_delt.z;

		theta_mag_square = ((p * p) + (q * q) + (r * r));
		scale_term = (0.5 - (theta_mag_square / 48.0)
				+ (theta_mag_square * theta_mag_square) / 3840.0);

		delta_quaternion_b.w = (1.0 - (theta_mag_square / 8.0)
				+ (theta_mag_square * theta_mag_square) / 384.0);

		delta_quaternion_b.x = scale_term * p;
		delta_quaternion_b.y = scale_term * q;
		delta_quaternion_b.z = scale_term * r;

		Quaternion_Multiply(&navigation->pure_solution.quaternion,
				&delta_quaternion_b, &updated_quaternion);

		Quaternion_Normalize(&updated_quaternion);
		navigation->pure_solution.quaternion = updated_quaternion;

		/*
		 * Earth rate compensation
		 */
		p = (navigation->rates.earth_rate_ned_rad_s.x
				+ navigation->rates.transport_rate_ned_rad_s.x)
				* NAVIGATION_UPDATE_PERIOD_S;
		q = (navigation->rates.earth_rate_ned_rad_s.y
				+ navigation->rates.transport_rate_ned_rad_s.y)
				* NAVIGATION_UPDATE_PERIOD_S;
		r = (navigation->rates.earth_rate_ned_rad_s.z
				+ navigation->rates.transport_rate_ned_rad_s.z)
				* NAVIGATION_UPDATE_PERIOD_S;

		theta_mag_square = ((p * p) + (q * q) + (r * r));
		scale_term = (0.5 - (theta_mag_square / 48.0)
				+ (theta_mag_square * theta_mag_square) / 3840.0);

		delta_quaternion_n.w = (1.0 - (theta_mag_square / 8.0)
				+ (theta_mag_square * theta_mag_square) / 384.0);

		delta_quaternion_n.x = scale_term * p;
		delta_quaternion_n.y = scale_term * q;
		delta_quaternion_n.z = scale_term * r;

		Quaternion_Inverse(&delta_quaternion_n, &delta_quaternion_n);

		Quaternion_Multiply(&delta_quaternion_n, &updated_quaternion,
				&navigation->pure_solution.quaternion);

		Quaternion_Normalize(&navigation->pure_solution.quaternion);

		/* Update DCM */
		Transform_QuaternionToDcm(&navigation->pure_solution.quaternion,
				&navigation->dcm_ned_to_body);

		Transform_QuaternionToEuler(&navigation->pure_solution.quaternion,
				&navigation->pure_solution.attitude);
	}
}

///* ==========================================================================
// * Velocity update
// * ========================================================================== */
//
//static void Navigation_UpdateVelocity(Navigation_t *navigation) {
////	Vector3_t specific_force_ned;
////
////	double coriolis_north;
////	double coriolis_east;
////	double coriolis_down;
//
//	double acceleration_north;
//	double acceleration_east;
//	double acceleration_down;
//
//	if (navigation != NULL) {
//		/*
//		 * Transform delta velocity from body to NED.
//		 *
//		 * DCM represents NED -> Body.
//		 *
//		 * Therefore:
//		 *
//		 * f_NED = C_B_NED^T * f_BODY
//		 */
////		Dcm_TransposeMultiplyVector(&navigation->dcm_ned_to_body,
////				&navigation->body_delta_velocity, &specific_force_ned);
//		/*
//		 * Coriolis + transport rate.
//		 *
//		 * omega = 2 * EarthRate + TransportRate
//		 */
//		coriolis_north = ((2.0 * navigation->earth_rate.east_rad_s)
//				+ navigation->transport_rate.east_rad_s)
//				* navigation->velocity.down_m_s;
//
//		coriolis_north -= ((2.0 * navigation->earth_rate.down_rad_s)
//				+ navigation->transport_rate.down_rad_s)
//				* navigation->velocity.east_m_s;
//
//		coriolis_east = ((2.0 * navigation->earth_rate.down_rad_s)
//				+ navigation->transport_rate.down_rad_s)
//				* navigation->velocity.north_m_s;
//
//		coriolis_east -= ((2.0 * navigation->earth_rate.north_rad_s)
//				+ navigation->transport_rate.north_rad_s)
//				* navigation->velocity.down_m_s;
//
//		coriolis_down = ((2.0 * navigation->earth_rate.north_rad_s)
//				+ navigation->transport_rate.north_rad_s)
//				* navigation->velocity.east_m_s;
//
//		coriolis_down -= ((2.0 * navigation->earth_rate.east_rad_s)
//				+ navigation->transport_rate.east_rad_s)
//				* navigation->velocity.north_m_s;
//
//		/*
//		 * NED velocity equation:
//		 *
//		 * Vdot = f_NED + g_NED - Omega x V
//		 *
//		 * where the above Coriolis terms represent
//		 * Omega x V.
//		 */
////		acceleration_north = specific_force_ned.x_m_s2
////				+ navigation->gravity.north_m_s2 - coriolis_north;
////
////		acceleration_east = specific_force_ned.y_m_s2
////				+ navigation->gravity.east_m_s2 - coriolis_east;
////
////		acceleration_down = specific_force_ned.z_m_s2
////				+ navigation->gravity.down_m_s2 - coriolis_down;
//		navigation->velocity.north_m_s += acceleration_north *
//		NAVIGATION_UPDATE_PERIOD_S;
//
//		navigation->velocity.east_m_s += acceleration_east *
//		NAVIGATION_UPDATE_PERIOD_S;
//
//		navigation->velocity.down_m_s += acceleration_down *
//		NAVIGATION_UPDATE_PERIOD_S;
//	}
//}
//
///* ==========================================================================
// * Position update
// * ========================================================================== */
//
//static void Navigation_UpdatePosition(Navigation_t *navigation) {
//	double latitude_dot;
//	double longitude_dot;
//	double altitude_dot;
//
//	double latitude;
//	double altitude;
//
//	double radius_meridian;
//	double radius_transverse;
//
//	if (navigation != NULL) {
//		latitude = navigation->position.latitude_rad;
//
//		altitude = navigation->position.altitude_m;
//
//		radius_meridian = navigation->radii.meridian_m;
//
//		radius_transverse = navigation->radii.transverse_m;
//
//		latitude_dot = navigation->velocity.north_m_s
//				/ (radius_meridian + altitude);
//
//		/*
//		 * Protect against division close to poles.
//		 */
//		if (fabs(cos(latitude)) > 1.0e-8) {
//			longitude_dot = navigation->velocity.east_m_s
//					/ ((radius_transverse + altitude) * cos(latitude));
//		} else {
//			longitude_dot = 0.0;
//		}
//
//		/*
//		 * NED convention:
//		 *
//		 * Down velocity positive.
//		 * Therefore:
//		 *
//		 * h_dot = -V_D
//		 */
//		altitude_dot = -navigation->velocity.down_m_s;
//
//		navigation->position.latitude_rad += latitude_dot *
//		NAVIGATION_UPDATE_PERIOD_S;
//
//		navigation->position.longitude_rad += longitude_dot *
//		NAVIGATION_UPDATE_PERIOD_S;
//
//		navigation->position.altitude_m += altitude_dot *
//		NAVIGATION_UPDATE_PERIOD_S;
//	}
//}
//
/* ==========================================================================
 * IMU buffer reset
 * ========================================================================== */

static void Navigation_ResetImuBuffer(Navigation_t *navigation) {
	uint32_t index;

	if (navigation != NULL) {
		navigation->imu_sample_count = 0U;

		for (index = 0U; index < NAVIGATION_IMU_SAMPLE_COUNT; index++) {
			Vector3_Zero(&navigation->imu_samples[index].gyro_rad_delt);
			Vector3_Zero(&navigation->imu_samples[index].accel_m_s_delt);
		}
	}
}

//	/* ==========================================================================
//	 * Navigation solution
//	 * ========================================================================== */
//
//	bool Navigation_GetSolution(const Navigation_t *navigation,
//			NavigationSolution_t *solution) {
//		bool valid;
//
//		valid = false;
//
//		if ((navigation != NULL) && (solution != NULL)
//				&& (navigation->initialized == true)) {
//			solution->position = navigation->position;
//
//			solution->velocity = navigation->velocity;
//
//			solution->attitude = navigation->attitude;
//
//			solution->valid = true;
//
//			valid = true;
//		}
//
//		return valid;
//	}
//
//	/* ==========================================================================
//	 * Update flag
//	 * ========================================================================== */
//
//	bool Navigation_IsUpdateAvailable(const Navigation_t *navigation) {
//		bool available;
//
//		available = false;
//
//		if (navigation != NULL) {
//			available = navigation->update_available;
//		}
//
//		return available;
//	}
//
//	void Navigation_ClearUpdateFlag(Navigation_t *navigation) {
//		if (navigation != NULL) {
//			navigation->update_available = false;
//		}
//	}
//

