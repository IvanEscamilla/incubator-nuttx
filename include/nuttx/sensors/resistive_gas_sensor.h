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

#define RGSIOC_BASE              (0x9100)
#define _RGSIOC(nr)              _IOC(RGSIOC_BASE, nr)
#define RGSIOC_READ_VOLT         _RGSIOC(0x0001) // Read sensor voltage data
#define RGSIOC_READ_LEVEL        _RGSIOC(0x0002) // Read sensor level data
#define RGSIOC_CALIB_READ_VOLT   _RGSIOC(0x0003) // Read calibration voltage data
#define RGSIOC_CALIB_READ_OFFSET _RGSIOC(0x0004) // Read calibration offset data
#define RGSIOC_DO_CALIBRATION    _RGSIOC(0x0005) // Do calibration

/* initialize function */

int resistive_gas_sensor_initialize(void);

int resistive_gas_sensor_uninitialize(void);

#endif /* __DRIVERS_PLATFORM_RESISTIVE_GAS_SENSOR_H */
