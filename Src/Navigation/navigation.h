/*
 * navigation.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef NAVIGATION_NAVIGATION_H_
#define NAVIGATION_NAVIGATION_H_

#include <stdbool.h>
#include <stdint.h>

#include "imu_types.h"
#include "navigation_types.h"

/*
 * Four IMU samples are received at 2.5 ms intervals.
 * One navigation update is therefore performed every 10 ms.
 */
#define NAVIGATION_IMU_SAMPLE_COUNT    (4U)

#define NAVIGATION_UPDATE_PERIOD_S     (0.010)
#define NAVIGATION_IMU_SAMPLE_PERIOD_S (0.0025)

#define CONE_K1             (214.0 / 315.0)
#define CONE_K2             (92.0  / 315.0)
#define CONE_K3             (54.0  / 315.0)

#define SCULL_K1_THETA      (54.0  / 105.0)
#define SCULL_K2_THETA      (92.0  / 105.0)
#define SCULL_K3_THETA      (214.0 / 105.0)

#define SCULL_K1_V          (54.0  / 105.0)
#define SCULL_K2_V          (29.0  / 105.0)
#define SCULL_K3_V          (214.0 / 105.0)

//void Navigation_Coning_Compensate(const ImuMeasurement_t samples[4],
//		ImuMeasurement_t *corrected);

void Navigation_Sculling_Compensate(const ImuMeasurement_t samples[4],
		ImuMeasurement_t *corrected);

void Navigation_Apply_Coning_Sculling(const Navigation_t *navigation,
		ImuMeasurement_t *corrected);

/**
 * @brief Initialize the navigation solution.
 *
 * Initializes position, velocity, attitude, gravity,
 * Earth rotation and IMU sample buffers.
 *
 * @param navigation Pointer to navigation state.
 */
void Navigation_Init_From_Mdl(const NavigationMdl_t *mdl_data,
		Navigation_t *navigation, NavigationSolution_t *solution);

/**
 * @brief Process one IMU sample.
 *
 * This function is called every 2.5 ms from the IMU ISR.
 *
 * Four consecutive IMU samples are accumulated and averaged.
 * Navigation mechanization is executed after the fourth sample.
 *
 * @param navigation Pointer to navigation state.
 * @param imu Pointer to IMU measurement.
 */
void Navigation_ImuUpdate(Navigation_t *navigation, const ImuMeasurement_t *imu);

/**
 * @brief Execute one complete INS mechanization update.
 *
 * Update sequence:
 *
 * 1. IMU preprocessing
 * 2. Gravity
 * 3. Earth/transport rates
 * 4. Attitude
 * 5. Velocity
 * 6. Position
 *
 * This function is normally called every 10 ms.
 *
 * @param navigation Pointer to navigation state.
 */
void Navigation_Update(Navigation_t *navigation);

/**
 * @brief Get the current navigation solution.
 *
 * @param navigation Pointer to navigation state.
 * @param solution Pointer to output navigation solution.
 *
 * @return true if solution is valid.
 * @return false otherwise.
 */
bool Navigation_GetSolution(const Navigation_t *navigation,
		NavigationSolution_t *solution);

/**
 * @brief Check whether a new 10 ms navigation solution is available.
 *
 * @param navigation Pointer to navigation state.
 *
 * @return true if a new solution is available.
 */
bool Navigation_IsUpdateAvailable(const Navigation_t *navigation);

/**
 * @brief Clear the navigation update flag.
 *
 * @param navigation Pointer to navigation state.
 */
void Navigation_ClearUpdateFlag(Navigation_t *navigation);

#endif /* NAVIGATION_NAVIGATION_H_ */
