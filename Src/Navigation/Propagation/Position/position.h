/*
 * position.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef NAVIGATION_PROPAGATION_POSITION_POSITION_H_
#define NAVIGATION_PROPAGATION_POSITION_POSITION_H_

#include "navigation_types.h"

void Position_Init(NavigationPosition_t *position);

void Position_Propagate(NavigationPosition_t *position,
		const NavigationVelocity_t *velocity, double dt_s);

#endif /* NAVIGATION_PROPAGATION_POSITION_POSITION_H_ */
