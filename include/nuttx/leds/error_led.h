/**
 * @file error_led.h
 * @author Luis Ivan Escamilla Estrada (liee0177@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __DRIVERS_PLATFORM_LEDS_ERROR_LED_H
#define __DRIVERS_PLATFORM_LEDS_ERROR_LED_H

#include <sys/ioctl.h>

/* Status led ioctl definitions **************************************/

#define ELIOC_BASE         (0xA000)
#define _ELIOC(nr)         _IOC(ELIOC_BASE, nr)
#define ELIOC_SET          _ELIOC(0x0001) // ON/OFF LED
#define ELIOC_SET_BRIGHT   _ELIOC(0x0002) // Ajustar brillo del LED
#define ELIOC_FADE_IN      _ELIOC(0x0003) // Activar efecto de fade in
#define ELIOC_FADE_OUT     _ELIOC(0x0004) // Activar efecto de fade out
#define ELIOC_BREATH_ANIM  _ELIOC(0x0005) // Activar animación de breath
#define ELIOC_CONN_ERROR   _ELIOC(0x0006) // Animacion Error de conexion
#define ELIOC_CALIB_ERROR  _ELIOC(0x0007) // Animacion Error de calibracion
#define ELIOC_BATT_ERROR   _ELIOC(0x0008) // Animacion Error de bateria
#define ELIOC_CONNECTED    _ELIOC(0x0009) // Connected animation
#define ELIOC_DISCONNECTED _ELIOC(0x000A) // Disconnect animation

/* initialize function */

int error_led_initialize(void);
int error_led_uninitialize(void);

#endif /* __DRIVERS_PLATFORM_LEDS_ERROR_LED_H */
