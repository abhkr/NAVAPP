/*
 * attitude.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef NAVIGATION_PROPAGATION_ATTITUDE_ATTITUDE_H_
#define NAVIGATION_PROPAGATION_ATTITUDE_ATTITUDE_H_

#include "navigation_types.h"

#define CONE_K1             (214.0 / 105.0)
#define CONE_K2             (92.0  / 105.0)
#define CONE_K3             (54.0  / 105.0)

#define SCULL_K1_THETA      (54.0  / 105.0)
#define SCULL_K2_THETA      (92.0  / 105.0)
#define SCULL_K3_THETA      (214.0 / 105.0)

#define SCULL_K1_V          (54.0  / 105.0)
#define SCULL_K2_V          (29.0  / 105.0)
#define SCULL_K3_V          (214.0 / 105.0)

void Attitude_Init(NavigationAttitude_t *attitude);

void Attitude_Propagate(NavigationAttitude_t *attitude,
		const NavigationImuIncrement_t *imu);

#endif /* NAVIGATION_PROPAGATION_ATTITUDE_ATTITUDE_H_ */
