/*
 * isr.h
 *
 *  Created on: 17-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef ISR_H_
#define ISR_H_

#include <stdbool.h>

#include "navigation.h"

/**
 * @brief Initialize application interrupt processing.
 *
 * @param navigation Pointer to navigation state.
 */
void Isr_Init(Navigation_t *navigation);

/**
 * @brief IMU interrupt service routine.
 *
 * Called every 2.5 ms.
 */
void Isr_Imu(void);

/**
 * @brief GPS interrupt service routine.
 *
 * Called when a GPS message is available.
 */
void Isr_Gps(void);

/**
 * @brief 1553B interrupt service routine.
 *
 * Called when a 1553B message is available.
 */
void Isr_Mil1553B(void);

#endif /* ISR_H_ */
