#include <stdbool.h>
#include <stdio.h>

#include "ins_types.h"
#include "navigation.h"
#include "isr.h"

static Navigation_t navigation;

static InsMode_t ins_mode;

static bool application_running;

static bool Main_ReadCommand(InsCommand_t *command);

static void Main_ExecuteCommand(InsCommand_t command);

static void Main_ProcessInsMode(void);

int main(void) {
	application_running = true;

	ins_mode = INS_MODE_IDLE;

	Navigation_Init(&navigation);

	Isr_Init(&navigation);

	/*
	 * Hardware/timer configuration will
	 * eventually enable:
	 *
	 * IMU : 2.5 ms
	 * GPS : 1 s
	 * 1553B : event driven
	 */

	while (application_running == true) {
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

//		HybridNavigation_Run(&navigation);

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

		input = getchar();

		while (getchar() != '\n') {
			/* Discard remaining input. */
		}

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

		application_running =
		false;

		break;

	case INS_COMMAND_NONE:

	default:

		break;
	}
}
