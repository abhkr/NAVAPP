/*
 * system_types.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef TYPES_SYSTEM_TYPES_H_
#define TYPES_SYSTEM_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Application operating mode.
 */
typedef enum {
	APP_MODE_IDLE = 0,
	APP_MODE_STATIC_ALIGNMENT,
	APP_MODE_TRANSFER_ALIGNMENT,
	APP_MODE_NAVIGATION,
	APP_MODE_EXIT

} AppMode_t;

/**
 * @brief User command.
 */
typedef enum {
	APP_COMMAND_NONE = 0,
	APP_COMMAND_STATIC_ALIGNMENT,
	APP_COMMAND_TRANSFER_ALIGNMENT,
	APP_COMMAND_NAVIGATION,
	APP_COMMAND_EXIT

} AppCommand_t;

/**
 * @brief Application events.
 */
typedef struct {
	bool imu_update;

	bool gps_update;

	bool navigation_update;

} AppEvents_t;

/**
 * @brief Application status.
 */
typedef struct {
	bool initialized;

	bool running;

	AppMode_t mode;

} AppStatus_t;

/**
 * @brief Generic timing information.
 */
typedef struct {
	uint64_t timestamp_us;

	uint64_t elapsed_us;

} SystemTime_t;

#endif /* TYPES_SYSTEM_TYPES_H_ */
