/*
 * app_config.h
 *
 *  Created on: 14-Aug-2026
 *      Author: ngarm-ins
 */

#ifndef APP_APP_CONFIG_H_
#define APP_APP_CONFIG_H_

/*
 * Application execution rates.
 */

#define APP_IMU_RATE_HZ                 (400U)
#define APP_IMU_PERIOD_MS               (2.5)

#define APP_NAVIGATION_RATE_HZ          (100U)
#define APP_NAVIGATION_PERIOD_MS        (10.0)

#define APP_GPS_RATE_HZ                 (1U)
#define APP_GPS_PERIOD_MS               (1000.0)

/*
 * Navigation integration period.
 */
#define APP_NAVIGATION_DT_S             (0.01)

/*
 * IMU sample period.
 */
#define APP_IMU_DT_S                    (0.0025)

/*
 * GPS update period.
 */
#define APP_GPS_DT_S                    (1.0)

#endif /* APP_CONFIG_H */#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * Application execution rates.
 */

#define APP_IMU_RATE_HZ                 (400U)
#define APP_IMU_PERIOD_MS               (2.5)

#define APP_NAVIGATION_RATE_HZ          (100U)
#define APP_NAVIGATION_PERIOD_MS        (10.0)

#define APP_GPS_RATE_HZ                 (1U)
#define APP_GPS_PERIOD_MS               (1000.0)

/*
 * Navigation integration period.
 */
#define APP_NAVIGATION_DT_S             (0.01)

/*
 * IMU sample period.
 */
#define APP_IMU_DT_S                    (0.0025)

/*
 * GPS update period.
 */
#define APP_GPS_DT_S                    (1.0)

#endif /* APP_APP_CONFIG_H_ */
