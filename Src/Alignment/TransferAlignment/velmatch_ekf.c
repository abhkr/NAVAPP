/*
 * velmatch_ekf.c
 *
 *  Created on: 09-Sept-2026
 *      Author: abhimanyu
 */

#include <math.h>
#include <stddef.h>
#include "wgs84.h"
#include "matrix.h"

#define WGS84_A   6378137.0
#define WGS84_E2  6.6943799901413165e-3

void ins_build_Fpp(double lat_rad, double height_m, double vN_mps,
		double vE_mps, Matrix3_t Fpp) {
	const double s = sin(lat_rad);
	const double c = cos(lat_rad);

	/*
	 * q = 1 - e^2 sin^2(phi)
	 */
	const double q = 1.0 - WGS84_E2 * s * s;
	const double sqrt_q = sqrt(q);

	/*
	 * WGS-84 radii of curvature
	 *
	 * RN: prime vertical radius
	 * RM: meridian radius
	 */
	const double RN =
	WGS84_A / sqrt_q;

	const double RM =
	WGS84_A * (1.0 - WGS84_E2) / (q * sqrt_q);

	/*
	 * Latitude derivatives of curvature radii.
	 *
	 * dRN/dphi =
	 * a e^2 sin(phi) cos(phi)
	 * --------------------------------
	 * (1 - e^2 sin^2(phi))^(3/2)
	 */
	const double dRN_dphi =
	WGS84_A * WGS84_E2 * s * c / (q * sqrt_q);

	/*
	 * dRM/dphi =
	 * 3 a (1-e^2) e^2 sin(phi) cos(phi)
	 * -----------------------------------
	 * (1 - e^2 sin^2(phi))^(5/2)
	 */
	const double dRM_dphi = 3.0 * WGS84_A * (1.0 - WGS84_E2) * WGS84_E2 * s * c
			/ (q * q * sqrt_q);

	/*
	 * Effective radii including altitude
	 */
	const double rN = RN + height_m;
	const double rM = RM + height_m;

	/*
	 * Zero matrix
	 */
	for (size_t i = 0; i < 3; ++i) {
		for (size_t j = 0; j < 3; ++j) {
			Fpp.m_data[i][j] = 0.0;
		}
	}

	/*
	 * ----------------------------------------------------
	 * Latitude equation:
	 *
	 * phi_dot = vN / (RM + h)
	 *
	 * d(phi_dot)/d(phi)
	 */
	Fpp.m_data[0][0] = -vN_mps * dRM_dphi / (rM * rM);

	/*
	 * d(phi_dot)/d(h)
	 */
	Fpp.m_data[0][2] = -vN_mps / (rM * rM);

	/*
	 * ----------------------------------------------------
	 * Longitude equation:
	 *
	 * lambda_dot = vE / ((RN + h) cos(phi))
	 *
	 * d(lambda_dot)/d(phi)
	 */
	Fpp.m_data[1][0] = (vE_mps / (rN * c)) * (tan(lat_rad) - dRN_dphi / rN);

	/*
	 * d(lambda_dot)/d(h)
	 */
	Fpp[1][2] = -vE_mps / (rN * rN * c);

	/*
	 * ----------------------------------------------------
	 * Height equation:
	 *
	 * h_dot = -vD
	 *
	 * No direct position-position coupling.
	 *
	 * Therefore row 2 remains zero.
	 * ----------------------------------------------------
	 */
}

void ins_build_Fpv(double lat_rad, double height_m, Matrix3_t Fpv) {
	const double s = sin(lat_rad);
	const double c = cos(lat_rad);

	/*
	 * q = 1 - e^2 sin^2(phi)
	 */
	const double q = 1.0 - WGS84_E2 * s * s;

	const double sqrt_q = sqrt(q);

	/*
	 * WGS-84 prime-vertical radius
	 *
	 * RN = a / sqrt(1 - e^2 sin^2(phi))
	 */
	const double RN =
	WGS84_A / sqrt_q;

	/*
	 * WGS-84 meridian radius
	 *
	 * RM = a(1-e^2)
	 *      -------------------------
	 *      (1-e^2 sin^2(phi))^(3/2)
	 */
	const double RM =
	WGS84_A * (1.0 - WGS84_E2) / (q * sqrt_q);

	/*
	 * Include altitude.
	 */
	const double rN = RN + height_m;
	const double rM = RM + height_m;

	/*
	 * Initialize to zero.
	 */
	for (size_t i = 0; i < 3; ++i) {
		for (size_t j = 0; j < 3; ++j) {
			Fpv.m_data[i][j] = 0.0;
		}
	}

	/*
	 * delta(phi_dot) / delta(vN)
	 */
	Fpv.m_data[0][0] = 1.0 / rM;

	/*
	 * delta(lambda_dot) / delta(vE)
	 */
	Fpv.m_data[1][1] = 1.0 / (rN * c);

	/*
	 * h_dot = -vD
	 */
	Fpv.m_data[2][2] = -1.0;
}

void build_Fvp_earth(double lat, double vN, double vE, double vD, Matrix3_t F) {
	const double s = sin(lat);
	const double c = cos(lat);

	F.m00 = -2.0 * WGS84_EARTH_ROTATION_RAD_S * vE * c;
	F.m01 = 0.0;
	F.m02 = 0.0;

	F.m10 = 2.0 * WGS84_EARTH_ROTATION_RAD_S * (vN * c - vD * s);

	F.m11 = 0.0;
	F.m12 = 0.0;

	F.m20 = 2.0 * WGS84_EARTH_ROTATION_RAD_S * vE * s;

	F.m21 = 0.0;
	F.m22 = 0.0;
}

void build_Fvp_transport(double lat, double h, double vN, double vE, double vD,
		Matrix3_t F) {
	const double s = sin(lat);
	const double c = cos(lat);

	const double tanLat = s / c;
	const double sec2Lat = 1.0 / (c * c);

	const double q = 1.0 - WGS84_E2 * s * s;

	const double sqrtQ = sqrt(q);

	const double RN =
	WGS84_A / sqrtQ;

	const double RM =
	WGS84_A * (1.0 - WGS84_E2) / (q * sqrtQ);

	const double dRN =
	WGS84_A * WGS84_E2 * s * c / (q * sqrtQ);

	const double dRM = 3.0 * WGS84_A * (1.0 - WGS84_E2) * WGS84_E2 * s * c
			/ (q * q * sqrtQ);

	const double rN = RN + h;
	const double rM = RM + h;

	/*
	 * Latitude column of Tp
	 */
	const double t1 = -vE * dRN / (rN * rN);

	const double t2 = vN * dRM / (rM * rM);

	const double t3 = -vE * (sec2Lat / rN - tanLat * dRN / (rN * rN));

	/*
	 * Height column of Tp
	 */
	const double u1 = -vE / (rN * rN);

	const double u2 = vN / (rM * rM);

	const double u3 = vE * tanLat / (rN * rN);

	/*
	 * Latitude column:
	 *
	 * [v x] * [t1 t2 t3]^T
	 */
	F.m00 = -vD * t2 + vE * t3;
	F.m10 = vD * t1 - vN * t3;
	F.m20 = -vE * t1 + vN * t2;

	/*
	 * Longitude column = zero
	 */
	F.m01 = 0.0;
	F.m11 = 0.0;
	F.m21 = 0.0;

	/*
	 * Height column:
	 *
	 * [v x] * [u1 u2 u3]^T
	 */
	F.m02 = -vD * u2 + vE * u3;
	F.m12 = vD * u1 - vN * u3;
	F.m22 = -vE * u1 + vN * u2;
}

void build_Fvp_gravity(double dg_dlat, double dg_dh, Matrix3_t F) {
	F.m00 = 0.0;
	F.m01 = 0.0;
	F.m02 = 0.0;

	F.m10 = 0.0;
	F.m11 = 0.0;
	F.m12 = 0.0;

	F.m20 = dg_dlat;
	F.m21 = 0.0;
	F.m22 = dg_dh;
}

void build_Fvp(double lat, double h, double vN, double vE, double vD,
		double dg_dlat, double dg_dh, Matrix3_t Fvp) {
	Matrix3_t Fe;
	Matrix3_t Ft;
	Matrix3_t Fg;

	build_Fvp_earth(lat, vN, vE, vD, Fe);

	build_Fvp_transport(lat, h, vN, vE, vD, Ft);

	build_Fvp_gravity(dg_dlat, dg_dh, Fg);

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			Fvp.m_data[i][j] = Fe.m_data[i][j] + Ft.m_data[i][j]
					+ Fg.m_data[i][j];
		}
	}
}

int ins_build_Fvv(double lat_rad, double height_m, double vN, double vE,
		double vD, Matrix3_t Fvv) {
	const double s = sin(lat_rad);
	const double c = cos(lat_rad);

	if (fabs(c) < 1.0e-8)
		return -1;

	const double tan_lat = s / c;

	/*
	 * WGS-84 curvature radii.
	 */
	const double q = 1.0 - WGS84_E2 * s * s;

	const double sqrt_q = sqrt(q);

	const double RN =
	WGS84_A / sqrt_q;

	const double RM =
	WGS84_A * (1.0 - WGS84_E2) / (q * sqrt_q);

	const double rN = RN + height_m;
	const double rM = RM + height_m;

	/*
	 * Omega = 2*omega_ie + omega_en
	 */
	const double OmegaN = 2.0 * WGS84_EARTH_ROTATION_RAD_S * c + vE / rN;

	const double OmegaE = -vN / rM;

	const double OmegaD = -2.0 * WGS84_EARTH_ROTATION_RAD_S * s
			- vE * tan_lat / rN;

	/*
	 * Fvv =
	 * -[Omega x] + [v x] Tv
	 *
	 * Explicit form.
	 */

	Fvv.m00 = vD / rM;

	Fvv.m01 = OmegaD - vE * tan_lat / rN;

	Fvv.m02 = -OmegaE;

	Fvv.m10 = -OmegaD;

	Fvv.m11 = vD / rN + vN * tan_lat / rN;

	Fvv.m12 = OmegaN;

	Fvv.m20 = OmegaE - vN / rM;

	Fvv.m21 = -OmegaN - vE / rN;

	Fvv.m22 = 0.0;

	return 0;
}

void ins_build_Fvtheta(const double fn[3], Matrix3_t Fvtheta) {
	const double fN = fn[0];
	const double fE = fn[1];
	const double fD = fn[2];

	Fvtheta.m00 = 0.0;
	Fvtheta.m01 = -fD;
	Fvtheta.m02 = fE;

	Fvtheta.m10 = fD;
	Fvtheta.m11 = 0.0;
	Fvtheta.m12 = -fN;

	Fvtheta.m20 = -fE;
	Fvtheta.m21 = fN;
	Fvtheta.m22 = 0.0;
}

void ins_build_Fvba(const Matrix3_t Cbn, Matrix3_t Fvba) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			Fvba.m_data[i][j] = Cbn[i][j];
		}
	}
}

int ins_build_Ftheta_p(double lat_rad, double height_m, double vN, double vE,
		Matrix3 Ftheta_p) {
	const double s = sin(lat_rad);
	const double c = cos(lat_rad);

	if (fabs(c) < 1.0e-8)
		return -1;

	const double tan_lat = s / c;
	const double sec2_lat = 1.0 / (c * c);

	/*
	 * WGS-84 curvature quantities
	 */
	const double q = 1.0 - WGS84_E2 * s * s;

	const double sqrt_q = sqrt(q);

	const double RN =
	WGS84_A / sqrt_q;

	const double RM =
	WGS84_A * (1.0 - WGS84_E2) / (q * sqrt_q);

	/*
	 * dRN / dphi
	 */
	const double dRN =
	WGS84_A * WGS84_E2 * s * c / (q * sqrt_q);

	/*
	 * dRM / dphi
	 */
	const double dRM = 3.0 * WGS84_A * (1.0 - WGS84_E2) * WGS84_E2 * s * c
			/ (q * q * sqrt_q);

	const double rN = RN + height_m;
	const double rM = RM + height_m;

	/*
	 * Ftheta_p = Ap + Tp
	 */

	Ftheta_p[0][0] = -WGS84_EARTH_ROTATION_RAD_S * s - vE * dRN / (rN * rN);

	Ftheta_p[0][1] = 0.0;

	Ftheta_p[0][2] = -vE / (rN * rN);

	Ftheta_p[1][0] = vN * dRM / (rM * rM);

	Ftheta_p[1][1] = 0.0;

	Ftheta_p[1][2] = vN / (rM * rM);

	Ftheta_p[2][0] = -WGS84_EARTH_ROTATION_RAD_S * c
			- vE * (sec2_lat / rN - tan_lat * dRN / (rN * rN));

	Ftheta_p[2][1] = 0.0;

	Ftheta_p[2][2] = vE * tan_lat / (rN * rN);

	return 0;
}

int ins_build_Ftheta_v(double lat_rad, double height_m, Matrix3 Ftheta_v) {
	const double s = sin(lat_rad);
	const double c = cos(lat_rad);

	if (fabs(c) < 1.0e-8)
		return -1;

	const double tan_lat = s / c;

	const double q = 1.0 - WGS84_E2 * s * s;

	const double sqrt_q = sqrt(q);

	const double RN =
	WGS84_A / sqrt_q;

	const double RM =
	WGS84_A * (1.0 - WGS84_E2) / (q * sqrt_q);

	const double rN = RN + height_m;
	const double rM = RM + height_m;

	Ftheta_v[0][0] = 0.0;
	Ftheta_v[0][1] = 1.0 / rN;
	Ftheta_v[0][2] = 0.0;

	Ftheta_v[1][0] = -1.0 / rM;
	Ftheta_v[1][1] = 0.0;
	Ftheta_v[1][2] = 0.0;

	Ftheta_v[2][0] = 0.0;
	Ftheta_v[2][1] = -tan_lat / rN;
	Ftheta_v[2][2] = 0.0;

	return 0;
}

int ins_build_Ftheta_theta(double lat_rad, double height_m, double vN,
		double vE, Matrix3 Ftheta_theta) {
	const double s = sin(lat_rad);
	const double c = cos(lat_rad);

	if (fabs(c) < 1.0e-8)
		return -1;

	const double tan_lat = s / c;

	/*
	 * WGS-84 radii of curvature
	 */
	const double q = 1.0 - WGS84_E2 * s * s;

	const double sqrt_q = sqrt(q);

	const double RN =
	WGS84_A / sqrt_q;

	const double RM =
	WGS84_A * (1.0 - WGS84_E2) / (q * sqrt_q);

	const double rN = RN + height_m;
	const double rM = RM + height_m;

	/*
	 * omega_in = omega_ie + omega_en
	 *
	 * NED components
	 */
	const double omegaN = WGS84_EARTH_ROTATION_RAD_S * c + vE / rN;

	const double omegaE = -vN / rM;

	const double omegaD = -WGS84_EARTH_ROTATION_RAD_S * s - vE * tan_lat / rN;

	/*
	 * Ftheta_theta = -[omega_in x]
	 */
	Ftheta_theta[0][0] = 0.0;
	Ftheta_theta[0][1] = omegaD;
	Ftheta_theta[0][2] = -omegaE;

	Ftheta_theta[1][0] = -omegaD;
	Ftheta_theta[1][1] = 0.0;
	Ftheta_theta[1][2] = omegaN;

	Ftheta_theta[2][0] = omegaE;
	Ftheta_theta[2][1] = -omegaN;
	Ftheta_theta[2][2] = 0.0;

	return 0;
}

void ins_build_Ftheta_bg(const Matrix3 Cbn, Matrix3 Ftheta_bg) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			Ftheta_bg[i][j] = -Cbn[i][j];
		}
	}
}

int ins_build_Fbg_gauss_markov(double tau_g, Matrix3 Fbg) {
	if (tau_g <= 0.0)
		return -1;

	const double lambda = -1.0 / tau_g;

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			Fbg[i][j] = 0.0;
		}

		Fbg[i][i] = lambda;
	}

	return 0;
}

int ins_build_Fbias_gauss_markov(const double tau[3], Matrix3 Fb) {
	for (int i = 0; i < 3; ++i) {
		if (tau[i] <= 0.0)
			return -1;
	}

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			Fb[i][j] = 0.0;
		}

		Fb[i][i] = -1.0 / tau[i];
	}

	return 0;
}

int ins_build_continuous_F(double lat_rad, double height_m,

double vN, double vE, double vD,

const double fn[3],

const Matrix3 Cbn,

const Matrix3 Gp,

const InsBiasConfig *bias_cfg,

Matrix15 F) {
	Matrix3 Fpp;
	Matrix3 Fpv;

	Matrix3 Fvp;
	Matrix3 Fvv;
	Matrix3 Fvtheta;
	Matrix3 Fvba;

	Matrix3 Ftheta_p;
	Matrix3 Ftheta_v;
	Matrix3 Ftheta_theta;
	Matrix3 Ftheta_bg;

	Matrix3 Fba;
	Matrix3 Fbg;

	/*
	 * Always zero complete matrix first.
	 */
	mat15_zero(F);

	/*
	 * --------------------------------------------------
	 * POSITION ROW
	 * --------------------------------------------------
	 */

	ins_build_Fpp(lat_rad, height_m, vN, vE, Fpp);

	ins_build_Fpv(lat_rad, height_m, Fpv);

	/*
	 * --------------------------------------------------
	 * VELOCITY ROW
	 * --------------------------------------------------
	 */

	if (ins_build_Fvp(lat_rad, height_m, vN, vE, vD, Gp, Fvp) != 0) {
		return -1;
	}

	if (ins_build_Fvv(lat_rad, height_m, vN, vE, vD, Fvv) != 0) {
		return -1;
	}

	ins_build_Fvtheta(fn, Fvtheta);

	ins_build_Fvba(Cbn, Fvba);

	/*
	 * --------------------------------------------------
	 * ATTITUDE ROW
	 * --------------------------------------------------
	 */

	if (ins_build_Ftheta_p(lat_rad, height_m, vN, vE, Ftheta_p) != 0) {
		return -1;
	}

	if (ins_build_Ftheta_v(lat_rad, height_m, Ftheta_v) != 0) {
		return -1;
	}

	if (ins_build_Ftheta_theta(lat_rad, height_m, vN, vE, Ftheta_theta) != 0) {
		return -1;
	}

	ins_build_Ftheta_bg(Cbn, Ftheta_bg);

	/*
	 * --------------------------------------------------
	 * BIAS ROWS
	 * --------------------------------------------------
	 */

	if (ins_build_bias_block(bias_cfg->accel_bias_model, bias_cfg->tau_accel,
			Fba) != 0) {
		return -1;
	}

	if (ins_build_bias_block(bias_cfg->gyro_bias_model, bias_cfg->tau_gyro, Fbg)
			!= 0) {
		return -1;
	}

	/*
	 * --------------------------------------------------
	 * INSERT BLOCKS
	 * --------------------------------------------------
	 */

	/* position */
	mat15_insert3(F, 0, 0, Fpp);
	mat15_insert3(F, 0, 3, Fpv);

	/* velocity */
	mat15_insert3(F, 3, 0, Fvp);
	mat15_insert3(F, 3, 3, Fvv);
	mat15_insert3(F, 3, 6, Fvtheta);
	mat15_insert3(F, 3, 9, Fvba);

	/* attitude */
	mat15_insert3(F, 6, 0, Ftheta_p);
	mat15_insert3(F, 6, 3, Ftheta_v);
	mat15_insert3(F, 6, 6, Ftheta_theta);
	mat15_insert3(F, 6, 12, Ftheta_bg);

	/* accel bias */
	mat15_insert3(F, 9, 9, Fba);

	/* gyro bias */
	mat15_insert3(F, 12, 12, Fbg);

	return 0;
}
