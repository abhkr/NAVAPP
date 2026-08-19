#include <stdbool.h>
#include <stdio.h>

#include "ins_types.h"
#include "navigation.h"
#include "math_constants.h"

static NavigationMdl_t mdl_data;

static Navigation_t navigation;

static NavigationSolution_t nav_solution_pure;

static InsMode_t ins_mode;

static bool Main_ReadCommand(InsCommand_t *command);

static void Main_ExecuteCommand(InsCommand_t command);

static void Main_ProcessInsMode(void);

static void Main_IMU_Isr(void) {
	navigation.rcnt++;

	printf("Time : %lf\n",
			(float64_t) (navigation.rcnt) * NAVIGATION_IMU_SAMPLE_PERIOD_S);
	printf("Attitude : %lf\n",
			nav_solution_pure.attitude.euler.yaw_rad * MATH_RAD_TO_DEG);
}

static void Main_Mdl(NavigationMdl_t *mdl) {
	mdl->position.latitude_rad = 17.257612 * MATH_DEG_TO_RAD;
	mdl->navigation_time = 10.0;
}

static void Main_Navigation(void) {
	ins_mode = INS_MODE_NAVIGATION;

	Navigation_Init_From_Mdl(&mdl_data, &navigation, &nav_solution_pure);

	while (navigation.rcnt <= ((uint32_t) (mdl_data.navigation_time * 400.0))) {
		Main_IMU_Isr();
	}

}

int main(void) {

	ins_mode = INS_MODE_IDLE;

	Main_Mdl(&mdl_data);

//	Isr_Init(&navigation);

	while (true) {
		InsCommand_t command;

		/*
		 * User command processing.
		 */
		if (Main_ReadCommand(&command) == true) {
			Main_ExecuteCommand(command);
		}

		/*
		 * Run selected INS mode.
		 */
		Main_ProcessInsMode();

		/*
		 * Process GPS/1553B events here
		 * rather than in their ISRs.
		 */
	}

	return 0;
}

static void Main_ProcessInsMode(void) {
	switch (ins_mode) {
	case INS_MODE_STATIC_ALIGNMENT:

//		StaticAlignment_Run(&navigation);

		break;

	case INS_MODE_TRANSFER_ALIGNMENT:

//		TransferAlignment_Run(&navigation);

		break;

	case INS_MODE_NAVIGATION:

		Main_Navigation();

		break;

	default:

		break;
	}
}

static bool Main_ReadCommand(InsCommand_t *command) {
	int input;
	bool valid;

	valid = false;

	if (command != NULL) {
		printf("\n");
		printf("INS COMMAND\n");
		printf("-----------\n");
		printf("1 - Static Alignment\n");
		printf("2 - Transfer Alignment\n");
		printf("3 - Navigation\n");
		printf("4 - Exit\n");
		printf("Select: ");

		fflush(stdout);

//		input = getchar();
//
//		while (getchar() != '\n') {
//			/* Discard remaining input. */
//		}
		input = '3';

		switch (input) {
		case '1':
			*command = INS_COMMAND_STATIC_ALIGNMENT;
			valid = true;
			break;

		case '2':
			*command = INS_COMMAND_TRANSFER_ALIGNMENT;
			valid = true;
			break;

		case '3':
			*command = INS_COMMAND_NAVIGATION;
			valid = true;
			break;

		case '4':
			*command = INS_COMMAND_EXIT;
			valid = true;
			break;

		default:
			*command = INS_COMMAND_NONE;
			break;
		}
	}

	return valid;
}

static void Main_ExecuteCommand(InsCommand_t command) {
	switch (command) {
	case INS_COMMAND_STATIC_ALIGNMENT:

		ins_mode = INS_MODE_STATIC_ALIGNMENT;

		printf("INS: Static Alignment\n");

		break;

	case INS_COMMAND_TRANSFER_ALIGNMENT:

		ins_mode = INS_MODE_TRANSFER_ALIGNMENT;

		printf("INS: Transfer Alignment\n");

		break;

	case INS_COMMAND_NAVIGATION:

		ins_mode = INS_MODE_NAVIGATION;

		printf("INS: Navigation\n");

		break;

	case INS_COMMAND_EXIT:

//		application_running =
//		false;

		break;

	case INS_COMMAND_NONE:

	default:

		break;
	}
}
