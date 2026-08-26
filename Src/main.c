#include <stdbool.h>
#include <stdio.h>

#include "ins_types.h"
#include "navigation.h"
#include "math_constants.h"
#include "imu_types.h"
#include "imu.h"
#include "transform.h"
#include "math_types.h"

static Navigation_t nav_state;

static ImuMeasurement_t imu;

static InsMode_t ins_mode;

static FILE *fpout = NULL;

//static void Main_Print_Euler(EulerAngles_t *euler) {
//	euler->roll_rad = 0.0;
////	printf(" %lf\t", euler->yaw_rad * MATH_RAD_TO_DEG);
////	printf("%lf\t", euler->pitch_rad * MATH_RAD_TO_DEG);
////	printf("%lf\t", euler->roll_rad * MATH_RAD_TO_DEG);
//}

static void Main_Print_Nav_Solution(FILE *fp, const Navigation_t *navstate) {

	fprintf(fp, "%lf\t",
			(float64_t) (navstate->rcnt) * NAVIGATION_IMU_SAMPLE_PERIOD_S);
//	Main_Print_Euler(&solution->attitude.euler);
//	printf("\n");

	fprintf(fp, "%0.10lf\t", navstate->imu_compensated.gyro_rad_delt.x);
	fprintf(fp, "%0.10lf\t", navstate->imu_compensated.gyro_rad_delt.y);
	fprintf(fp, "%0.10lf\t", navstate->imu_compensated.gyro_rad_delt.z);

	fprintf(fp, "%0.10lf\t", navstate->imu_compensated.accel_m_s_delt.x);
	fprintf(fp, "%0.10lf\t", navstate->imu_compensated.accel_m_s_delt.y);
	fprintf(fp, "%0.10lf\t", navstate->imu_compensated.accel_m_s_delt.z);

	fprintf(fp, "%0.10lf\t",
			navstate->pure_solution.attitude.yaw_rad * MATH_RAD_TO_DEG);
	fprintf(fp, "%0.10lf\t",
			navstate->pure_solution.attitude.pitch_rad * MATH_RAD_TO_DEG);
	fprintf(fp, "%0.10lf\t",
			navstate->pure_solution.attitude.roll_rad * MATH_RAD_TO_DEG);

	fprintf(fp, "\n");
}

static void Main_IMU_Isr(void) {
	nav_state.rcnt++;
	Imu_AcquireStatic(&imu, nav_state.pure_solution.position.latitude_rad,
	NAVIGATION_IMU_SAMPLE_PERIOD_S, &nav_state.dcm_ned_to_body);

	nav_state.imu_samples[nav_state.imu_sample_count] = imu;
	nav_state.imu_sample_count++;

	if (nav_state.imu_sample_count == NAVIGATION_IMU_SAMPLE_COUNT) {
		Navigation_Apply_Coning_Sculling(&nav_state);
		Navigation_UpdateAttitude(&nav_state);

		Main_Print_Nav_Solution(fpout, &nav_state);
		nav_state.imu_sample_count = 0U;
	}

}

static void Main_Mdl(NavigationMdl_t *mdl) {
	mdl->position.latitude_rad = 17.257612 * MATH_DEG_TO_RAD;
	mdl->position.longitude_rad = 78.511145 * MATH_DEG_TO_RAD;
	mdl->position.altitude_m = 510.0;

	mdl->attitude.yaw_rad = 10.0 * MATH_DEG_TO_RAD;
	mdl->attitude.pitch_rad = 0.0 * MATH_DEG_TO_RAD;
	mdl->attitude.roll_rad = 0.0 * MATH_DEG_TO_RAD;

	mdl->velocity.north_m_s = 0.0;
	mdl->velocity.east_m_s = 0.0;
	mdl->velocity.down_m_s = 0.0;

	mdl->leveling_time = 5.0;
	mdl->navigation_time = 10.0;
}

static void Main_Navigation(NavigationMdl_t *mdl_data) {
	ins_mode = INS_MODE_NAVIGATION;

	Navigation_Init_From_Mdl(mdl_data, &nav_state);

	while (nav_state.rcnt <= ((uint32_t) (mdl_data->navigation_time * 400.0))) {
		Main_IMU_Isr();
	}
}

int main(void) {
	fpout = fopen("nav.txt", "w");
	if (fpout == NULL) {
		fprintf(stderr, "Unable to open files \n");
		return -1;
	}

	NavigationMdl_t mdl_data;
	ins_mode = INS_MODE_IDLE;

	Main_Mdl(&mdl_data);

	Main_Navigation(&mdl_data);

	printf("completed\n");

	fclose(fpout);

	return 0;
}

