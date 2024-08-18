/**
 * @file status_led.h
 * @author Luis Ivan Escamilla Estrada (liee0177@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __DRIVERS_PLATFORM_LEDS_STATUS_LED_H
#define __DRIVERS_PLATFORM_LEDS_STATUS_LED_H

#include <sys/ioctl.h>

/* Status led ioctl definitions **************************************/

#define SLIOC_BASE         (0x9000)
#define _SLIOC(nr)         _IOC(SLIOC_BASE, nr)
#define SLIOC_SET          _SLIOC(0x0001) // ON/OFF LED
#define SLIOC_SET_BRIGHT   _SLIOC(0x0002) // Ajustar brillo del LED
#define SLIOC_FADE_IN      _SLIOC(0x0003) // Activar efecto de fade in
#define SLIOC_FADE_OUT     _SLIOC(0x0004) // Activar efecto de fade out
#define SLIOC_BREATH_ANIM  _SLIOC(0x0005) // Activar animación de breath
#define SLIOC_CONN_ERROR   _SLIOC(0x0006) // Animacion Error de conexion
#define SLIOC_CALIB_ERROR  _SLIOC(0x0007) // Animacion Error de calibracion
#define SLIOC_BATT_ERROR   _SLIOC(0x0008) // Animacion Error de bateria
#define SLIOC_CONNECTED    _SLIOC(0x0009) // Connected animation
#define SLIOC_DISCONNECTED _SLIOC(0x000A) // Disconnect animation

/* initialize function */

int status_led_initialize();
int status_led_uninitialize(void);

#endif /* __DRIVERS_PLATFORM_LEDS_STATUS_LED_H */
