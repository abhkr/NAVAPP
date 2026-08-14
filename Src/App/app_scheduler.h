/*
 * app_scheduler.h
 *
 *  Created on: 14-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef APP_APP_SCHEDULER_H_
#define APP_APP_SCHEDULER_H_

#include <stdbool.h>

/**
 * @brief Scheduler events.
 */
typedef struct {
	bool imu_400hz;
	bool navigation_100hz;
	bool gps_1hz;
	bool output_100hz;
} AppSchedulerEvents_t;

/**
 * @brief Initialize scheduler.
 */
void AppScheduler_Init(void);

/**
 * @brief Generate periodic application events.
 *
 * Called from the system timer ISR.
 */
void AppScheduler_Tick(void);

/**
 * @brief Get and clear pending scheduler events.
 */
void AppScheduler_GetEvents(AppSchedulerEvents_t *events);

#endif /* APP_APP_SCHEDULER_H_ */
