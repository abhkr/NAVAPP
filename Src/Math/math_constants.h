/*
 * math_constants.h
 *
 *  Created on: 11-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef MATH_CONSTANTS_H_
#define MATH_CONSTANTS_H_

#include "math_types.h"
#include <math.h>

#define MATH_PI              (4.0 * atan(1.0))

#define MATH_DEG_TO_RAD      (MATH_PI/180.0)
#define MATH_RAD_TO_DEG      (180.0/MATH_PI)

#define MATH_EPSILON         (1.0e-12)

#endif /* MATH_CONSTANTS_H_ */
