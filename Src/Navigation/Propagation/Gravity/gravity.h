/*
 * gravity.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef NAVIGATION_PROPAGATION_GRAVITY_GRAVITY_H_
#define NAVIGATION_PROPAGATION_GRAVITY_GRAVITY_H_

#include "navigation_types.h"

void Gravity_Init(NavigationGravity_t *gravity);

void Gravity_Update(NavigationGravity_t *gravity,
		const NavigationPosition_t *position);

#endif /* NAVIGATION_PROPAGATION_GRAVITY_GRAVITY_H_ */
