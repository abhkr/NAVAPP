#include <stdbool.h>
#include <stdio.h>

#include "ins_types.h"
#include "navigation.h"
#include "math_constants.h"
#include "imu_types.h"
#include "imu.h"

static NavigationMdl_t mdl_data;

static Navigation_t nav_state;

static ImuMeasurement_t imu;

static InsMode_t ins_mode;

//static void Main_Print_Euler(EulerAngles_t *euler) {
//	euler->roll_rad = 0.0;
////	printf(" %lf\t", euler->yaw_rad * MATH_RAD_TO_DEG);
////	printf("%lf\t", euler->pitch_rad * MATH_RAD_TO_DEG);
////	printf("%lf\t", euler->roll_rad * MATH_RAD_TO_DEG);
//}

static void Main_Print_Nav_Solution(const Navigation_t *navstate) {

	printf("%lf\t",
			(float64_t) (navstate->rcnt) * NAVIGATION_IMU_SAMPLE_PERIOD_S);
//	Main_Print_Euler(&solution->attitude.euler);
//	printf("\n");

	printf("%0.10lf\t", navstate->imu_compensated.gyro_rad_delt.x);
	printf("%0.10lf\t", navstate->imu_compensated.gyro_rad_delt.y);
	printf("%0.10lf\t", navstate->imu_compensated.gyro_rad_delt.z);

	printf("%0.10lf\t", navstate->imu_compensated.accel_m_s_delt.x);
	printf("%0.10lf\t", navstate->imu_compensated.accel_m_s_delt.y);
	printf("%0.10lf\t", navstate->imu_compensated.accel_m_s_delt.z);
	printf("\n");
}

static void Main_IMU_Isr(void) {
	nav_state.rcnt++;
	Imu_AcquireStatic(&imu, nav_state.pure_solution.position.latitude_rad,
	NAVIGATION_IMU_SAMPLE_PERIOD_S, &nav_state.dcm_ned_to_body);

	nav_state.imu_samples[nav_state.imu_sample_count] = imu;
	nav_state.imu_sample_count++;

	if (nav_state.imu_sample_count == NAVIGATION_IMU_SAMPLE_COUNT) {
		Navigation_Apply_Coning_Sculling(&nav_state);
		Main_Print_Nav_Solution(&nav_state);
		nav_state.imu_sample_count = 0U;
	}
}

static void Main_Mdl(NavigationMdl_t *mdl) {
	mdl->position.latitude_rad = 17.257612 * MATH_DEG_TO_RAD;
	mdl->navigation_time = 10.0;
}

static void Main_Navigation(void) {
	ins_mode = INS_MODE_NAVIGATION;

	Navigation_Init_From_Mdl(&mdl_data, &nav_state);

	while (nav_state.rcnt <= ((uint32_t) (mdl_data.navigation_time * 400.0))) {
		Main_IMU_Isr();
	}
}

int main(void) {

	ins_mode = INS_MODE_IDLE;

	Main_Mdl(&mdl_data);

	Main_Navigation();

	return 0;
}

