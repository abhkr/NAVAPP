/*
 * mil1553b_driver.h
 *
 *  Created on: 12-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef MIL1553B_DRIVER_H_
#define MIL1553B_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief MIL-STD-1553B driver status.
 */
typedef enum {
	MIL1553B_STATUS_OK = 0,
	MIL1553B_STATUS_NULL_POINTER,
	MIL1553B_STATUS_BUSY,
	MIL1553B_STATUS_TIMEOUT,
	MIL1553B_STATUS_BUS_ERROR,
	MIL1553B_STATUS_INVALID_ARGUMENT
} Mil1553bStatus_t;

/**
 * @brief MIL-STD-1553B driver configuration.
 */
typedef struct {
	uint8_t terminal_address;
	bool parity_enabled;
} Mil1553bDriverConfig_t;

/**
 * @brief MIL-STD-1553B driver state.
 */
typedef struct {
	Mil1553bDriverConfig_t config;

	bool initialized;
	bool busy;
} Mil1553bDriver_t;

/**
 * @brief Initialize MIL-STD-1553B driver.
 */
Mil1553bStatus_t Mil1553bDriver_Init(Mil1553bDriver_t *driver,
		const Mil1553bDriverConfig_t *config);

/**
 * @brief Transmit a data word.
 */
Mil1553bStatus_t Mil1553bDriver_TransmitWord(Mil1553bDriver_t *driver,
		uint16_t data);

/**
 * @brief Transmit multiple data words.
 */
Mil1553bStatus_t Mil1553bDriver_TransmitWords(Mil1553bDriver_t *driver,
		const uint16_t *data, uint8_t word_count);

/**
 * @brief Receive a data word.
 */
Mil1553bStatus_t Mil1553bDriver_ReceiveWord(Mil1553bDriver_t *driver,
		uint16_t *data);

/**
 * @brief Check whether driver is busy.
 */
bool Mil1553bDriver_IsBusy(const Mil1553bDriver_t *driver);

#endif /* MIL1553B_DRIVER_H_ */
