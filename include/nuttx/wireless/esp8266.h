/**
 * @file esp8266.h
 * @author Luis Ivan Escamilla Estrada (liee0177@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-01-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __DRIVERS_PLATFORM_WIRELESS_ESP8266_H
#define __DRIVERS_PLATFORM_WIRELESS_ESP8266_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <net/if.h>
#include <netinet/in.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/ioctl.h>

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#define ESP8266_FIRST       (WL_FIRST + WL_NCMDS)
#define ESP8266_NCMDS       9

#define ESP8266_IOC_CONNECT _WLCIOC(ESP8266_FIRST + 0)
#define ESP8266_IOC_SEND    _WLCIOC(ESP8266_FIRST + 1)
#define ESP8266_IOC_RECV    _WLCIOC(ESP8266_FIRST + 2)
#define ESP8266_IOC_CLOSE   _WLCIOC(ESP8266_FIRST + 3)
#define ESP8266_IOC_BIND    _WLCIOC(ESP8266_FIRST + 4)
#define ESP8266_IOC_ACCEPT  _WLCIOC(ESP8266_FIRST + 5)
#define ESP8266_IOC_ASSOC   _WLCIOC(ESP8266_FIRST + 6)
#define ESP8266_IOC_IFREQ   _WLCIOC(ESP8266_FIRST + 7)
#define ESP8266_IOC_NAME    _WLCIOC(ESP8266_FIRST + 8)

// TODO: is this supported on esp8266
#define DISASSOCIATION_CID  ('x')

    /* NOTE: do not forget to update include/nuttx/wireless/ioctl.h */

    struct esp8266_connect_msg
    {
        char cid;
        uint8_t type;
        uint16_t lport;
        char addr[17];
        char port[6];
    };

    struct esp8266_bind_msg
    {
        char cid;
        uint8_t type;
        bool is_tcp;
        char port[6];
    };

    struct esp8266_accept_msg
    {
        struct sockaddr_in addr;
        char cid;
        uint8_t type;
    };

    struct esp8266_send_msg
    {
        struct sockaddr_in addr; /* used for udp */
        bool is_tcp;
        char cid;
        uint8_t type;
        FAR uint8_t *buf;
        uint16_t len;
    };

    struct esp8266_recv_msg
    {
        struct sockaddr_in addr; /* used for udp */
        bool is_tcp;
        char cid;
        uint8_t type;
        FAR uint8_t *buf;
        uint16_t len;    /* actual buffer length */
        uint16_t reqlen; /* requested size */
        int32_t flags;   /* MSG_* flags */
    };

    struct esp8266_close_msg
    {
        char cid;
        uint8_t type;
    };

    struct esp8266_assoc_msg
    {
        FAR char *ssid;
        FAR char *key;
        uint8_t mode;
        uint8_t ch;
    };

    struct esp8266_ifreq_msg
    {
        uint32_t cmd;
        struct ifreq ifr;
    };

    struct esp8266_name_msg
    {
        struct sockaddr_in addr;
        bool local;
        char cid;
    };

    struct esp8266_lower_s
    {
        int (*attach)(xcpt_t handler, FAR void *arg);
        void (*enable)(void);
        void (*disable)(void);
        uint32_t (*dready)(int *);
        void (*rts)(bool);
        void (*set_cts_dir)(bool);
        void (*reset)(bool);
    };

    FAR void *esp8266_register(FAR const char *devpath,
                               FAR const struct esp8266_lower_s *lower);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __DRIVERS_PLATFORM_WIRELESS_ESP8266_H */
