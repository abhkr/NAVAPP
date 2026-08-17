/*
 * ins_types.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef TYPES_INS_TYPES_H_
#define TYPES_INS_TYPES_H_

typedef enum {
	INS_MODE_IDLE = 0,
	INS_MODE_STATIC_ALIGNMENT,
	INS_MODE_TRANSFER_ALIGNMENT,
	INS_MODE_NAVIGATION

} InsMode_t;

/**
 * @brief Command received from the user.
 */
typedef enum {
	INS_COMMAND_NONE = 0U,
	INS_COMMAND_STATIC_ALIGNMENT,
	INS_COMMAND_TRANSFER_ALIGNMENT,
	INS_COMMAND_NAVIGATION,
	INS_COMMAND_EXIT

} InsCommand_t;

#endif /* TYPES_INS_TYPES_H_ */
