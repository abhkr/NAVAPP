/*
 * isr.c
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#include "isr.h"

#include <stddef.h>

#include "imu.h"

static Navigation_t *isr_navigation;

void Isr_Init(Navigation_t *navigation) {
	isr_navigation = navigation;
}

void Isr_Imu(void) {
	ImuMeasurement_t measurement;
	bool valid;

	if (isr_navigation != NULL) {
		valid = Imu_Acquire(&measurement);

		if (valid == true) {
			Navigation_ImuIsr(isr_navigation, &measurement);
		}
	}
}

