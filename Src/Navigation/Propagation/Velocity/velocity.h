/*
 * velocity.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef NAVIGATION_PROPAGATION_VELOCITY_VELOCITY_H_
#define NAVIGATION_PROPAGATION_VELOCITY_VELOCITY_H_

#include "navigation_types.h"

void Velocity_Init(NavigationVelocity_t *velocity);

void Velocity_Propagate(NavigationVelocity_t *velocity,
		const NavigationAttitude_t *attitude,
		const NavigationGravity_t *gravity, const NavigationImuIncrement_t *imu);

#endif /* NAVIGATION_PROPAGATION_VELOCITY_VELOCITY_H_ */
