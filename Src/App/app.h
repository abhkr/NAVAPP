/*
 * app.h
 *
 *  Created on: 14-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef APP_APP_H_
#define APP_APP_H_

#include <stdbool.h>
#include <stdint.h>

#include "imu.h"

/**
 * @brief Application operating mode.
 */
typedef enum {
	APP_STATE_INIT = 0,
	APP_STATE_STATIC_ALIGNMENT,
	APP_STATE_TRANSFER_ALIGNMENT,
	APP_STATE_NAVIGATION,
	APP_STATE_FAULT
} AppState_t;

/**
 * @brief Application event flags.
 */
typedef struct {
	bool imu_update;
	bool navigation_update;
	bool gps_update;
	bool navigation_output;
} AppEvents_t;

/**
 * @brief Application state.
 */
typedef struct {
	AppState_t state;

	AppEvents_t events;

	ImuMeasurement_t imu_measurement;
//	GpsMeasurement_t gps_measurement;

	NavigationSolution_t navigation_solution;

//	Navigation_t navigation;

	bool initialized;
} App_t;

/**
 * @brief Initialize application.
 */
void App_Init(App_t *app);

/**
 * @brief Execute application processing.
 *
 * This function is called from the main loop.
 */
void App_Run(App_t *app);

/**
 * @brief Notify application that an IMU sample is available.
 *
 * This function may be called from an ISR.
 */
void App_NotifyImu(App_t *app);

/**
 * @brief Notify application that a GPS measurement is available.
 *
 * This function may be called from an ISR.
 */
void App_NotifyGps(App_t *app);

/**
 * @brief Get current application state.
 */
AppState_t App_GetState(const App_t *app);

#endif /* APP_APP_H_ */
