/*
 * attitude.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef NAVIGATION_PROPAGATION_ATTITUDE_ATTITUDE_H_
#define NAVIGATION_PROPAGATION_ATTITUDE_ATTITUDE_H_

#include "navigation_types.h"

void Attitude_Init(NavigationAttitude_t *attitude);

void Attitude_Propagate(NavigationAttitude_t *attitude,
		const NavigationImuIncrement_t *imu);

#endif /* NAVIGATION_PROPAGATION_ATTITUDE_ATTITUDE_H_ */
