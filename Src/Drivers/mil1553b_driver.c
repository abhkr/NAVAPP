/*
 * mil1553b_driver.c
 *
 *  Created on: 12-Aug-2026
 *      Author: ngarm-ins
 */

#include "mil1553b_driver.h"

#include <stdlib.h>

#define MIL1553B_MAX_DATA_WORDS    (32U)

Mil1553bStatus_t Mil1553bDriver_Init(Mil1553bDriver_t *driver,
		const Mil1553bDriverConfig_t *config) {
	Mil1553bStatus_t status;

	status = MIL1553B_STATUS_NULL_POINTER;

	if ((driver != NULL) && (config != NULL)) {
		driver->config = *config;

		driver->initialized = true;
		driver->busy = false;

		/*
		 * Hardware initialization will be performed here.
		 *
		 * This should eventually call BSP/1553B hardware functions
		 * rather than accessing MCU registers directly.
		 */

		status = MIL1553B_STATUS_OK;
	}

	return status;
}

Mil1553bStatus_t Mil1553bDriver_TransmitWord(Mil1553bDriver_t *driver,
		uint16_t data) {
	Mil1553bStatus_t status;

	status = MIL1553B_STATUS_NULL_POINTER;

	if (driver != NULL) {
		if (driver->initialized == false) {
			status = MIL1553B_STATUS_INVALID_ARGUMENT;
		} else if (driver->busy == true) {
			status = MIL1553B_STATUS_BUSY;
		} else {
			driver->busy = true;

			/*
			 * TODO:
			 *
			 * Write data to the actual 1553B controller.
			 */
			(void) data;

			driver->busy = false;

			status = MIL1553B_STATUS_OK;
		}
	}

	return status;
}

Mil1553bStatus_t Mil1553bDriver_TransmitWords(Mil1553bDriver_t *driver,
		const uint16_t *data, uint8_t word_count) {
	Mil1553bStatus_t status;
	uint8_t index;

	status = MIL1553B_STATUS_NULL_POINTER;

	if ((driver != NULL) && (data != NULL)) {
		if (word_count > MIL1553B_MAX_DATA_WORDS) {
			status = MIL1553B_STATUS_INVALID_ARGUMENT;
		} else {
			status = MIL1553B_STATUS_OK;

			for (index = 0U; index < word_count; index++) {
				status = Mil1553bDriver_TransmitWord(driver, data[index]);

				if (status != MIL1553B_STATUS_OK) {
					break;
				}
			}
		}
	}

	return status;
}

Mil1553bStatus_t Mil1553bDriver_ReceiveWord(Mil1553bDriver_t *driver,
		uint16_t *data) {
	Mil1553bStatus_t status;

	status = MIL1553B_STATUS_NULL_POINTER;

	if ((driver != NULL) && (data != NULL)) {
		/*
		 * TODO:
		 *
		 * Read actual 1553B controller hardware.
		 */

		*data = 0U;

		status = MIL1553B_STATUS_OK;
	}

	return status;
}

bool Mil1553bDriver_IsBusy(const Mil1553bDriver_t *driver) {
	bool status;

	status = false;

	if (driver != NULL) {
		status = driver->busy;
	}

	return status;
}
