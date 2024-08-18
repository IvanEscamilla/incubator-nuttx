/**
 * @file resistive_gas_sensor.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-07-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __DRIVERS_PLATFORM_RESISTIVE_GAS_SENSOR_H
#define __DRIVERS_PLATFORM_RESISTIVE_GAS_SENSOR_H

#include <sys/ioctl.h>

/* Status led ioctl definitions **************************************/

#define RGSIOC_BASE       (0x9100)
#define _RGSIOC(nr)       _IOC(RGSIOC_BASE, nr)
#define RGSIOC_READ       _RGSIOC(0x0001) // Read sensor data
#define RGSIOC_CALIB_READ _RGSIOC(0x0002) // Read calibration data

/* initialize function */

int resistive_gas_sensor_initialize(void);

int resistive_gas_sensor_uninitialize(void);

#endif /* __DRIVERS_PLATFORM_RESISTIVE_GAS_SENSOR_H */
