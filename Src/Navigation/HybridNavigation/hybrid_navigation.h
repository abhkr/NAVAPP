/*
 * hybrid_navigation.h
 *
 *  Created on: 12-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef HYBRID_NAVIGATION_H_
#define HYBRID_NAVIGATION_H_

#include <stdbool.h>

#include "navigation_types.h"

/**
 * @brief Initialize hybrid navigation.
 *
 * @param hybrid Hybrid navigation state.
 */
void HybridNavigation_Init(HybridNavigationState_t *hybrid);

/**
 * @brief Process GPS measurement.
 *
 * Intended execution rate: 1 Hz.
 *
 * @param hybrid Hybrid navigation state.
 * @param pure_navigation Pure navigation state.
 * @param gps GPS measurement.
 *
 * @return true if GPS update was accepted.
 */
bool HybridNavigation_Update(HybridNavigationState_t *hybrid,
		PureNavigationState_t *pure_navigation,
		const NavigationGpsMeasurement_t *gps);

/**
 * @brief Get current hybrid navigation error state.
 *
 * @param hybrid Hybrid navigation state.
 * @param error_state Output error state.
 *
 * @return true if error state is valid.
 */
bool HybridNavigation_GetErrorState(const HybridNavigationState_t *hybrid,
		NavigationErrorState_t *error_state);

#endif /* HYBRID_NAVIGATION_H_ */
