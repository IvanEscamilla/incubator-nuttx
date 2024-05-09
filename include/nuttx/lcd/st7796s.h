/****************************************************************************
 * drivers/platform/lcd/st7796s.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __DRIVERS_PLATFORM_LCD_ST7796S_H
#define __DRIVERS_PLATFORM_LCD_ST7796S_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ST7796S ID code */

#define ST7796S_DEVICE_CODE 0x9486

/* ST7796S LCD Register Addresses *******************************************/

#define ST7796S_NOP 0x00        /* Nop */
#define ST7796S_SOFT_RESET 0x01 /* Soft Reset */
#define ST7796S_READ_DISPLAY_ID_INFORMATION                                    \
    0x04 /* Read Display Identification Information */
#define ST7796S_READ_NUMBER_OF_THE_ERRORS_ON_DSI                               \
    0x05                                 /* Read Number Of The Errors On Dsi */
#define ST7796S_READ_DISPLAY_STATUS 0x09 /* Read Display Status */
#define ST7796S_READ_DISPLAY_POWER_MODE 0x0A   /* Read Display Power Mode */
#define ST7796S_READ_DISPLAY_MADCTL 0x0B       /* Read Display Madctl */
#define ST7796S_READ_DISPLAY_PIXEL_FORMAT 0x0C /* Read Display Pixel Format */
#define ST7796S_READ_DISPLAY_IMAGE_MODE 0x0D   /* Read Display Image Mode */
#define ST7796S_READ_DISPLAY_SIGNAL_MODE 0x0E  /* Read Display Signal Mode */
#define ST7796S_READ_DISPLAY_SELF_DIAG_RESULT                                  \
    0x0F                             /* Read Display Self-diagnostic Result */
#define ST7796S_SLEEP_IN 0x10        /* Sleep In */
#define ST7796S_SLEEP_OUT 0x11       /* Sleep Out */
#define ST7796S_PARTIAL_MODE_ON 0x12 /* Partial Mode On */
#define ST7796S_NORMAL_DISPLAY_MODE_ON 0x13 /* Normal Display Mode On */
#define ST7796S_DISPLAY_INVERSION_OFF 0x20  /* Display Inversion Off */
#define ST7796S_DISPLAY_INVERSION_ON 0x21   /* Display Inversion On */
#define ST7796S_DISPLAY_OFF 0x28            /* Display Off */
#define ST7796S_DISPLAY_ON 0x29             /* Display On */
#define ST7796S_COLUMN_ADDRESS_SET 0x2A     /* Column Address Set */
#define ST7796S_PAGE_ADDRESS_SET 0x2B       /* Page Address Set */
#define ST7796S_MEMORY_WRITE 0x2C           /* Memory Write */
#define ST7796S_MEMORY_READ 0x2E            /* Memory Read */
#define ST7796S_PARTIAL_AREA 0x30           /* Partial Area */
#define ST7796S_VERTICAL_SCROLLING_DEFINITION                                  \
    0x33                                     /* Vertical Scrolling Definition */
#define ST7796S_TEARING_EFFECT_LINE_OFF 0x34 /* Tearing Effect Line Off */
#define ST7796S_TEARING_EFFECT_LINE_ON 0x35  /* Tearing Effect Line On */
#define ST7796S_MEMORY_ACCESS_CTRL 0x36      /* Memory Access Control */
#define ST7796S_VERTICAL_SCROLLING_START_ADDRESS                               \
    0x37                           /* Vertical Scrolling Start Address */
#define ST7796S_IDLE_MODE_OFF 0x38 /* Idle Mode Off */
#define ST7796S_IDLE_MODE_ON 0x39  /* Idle Mode On */
#define ST7796S_INTERFACE_PIXEL_FORMAT 0x3A /* Interface Pixel Format */
#define ST7796S_MEMORY_WRITE_CONTINUE 0x3C  /* Memory Write Continue */
#define ST7796S_MEMORY_READ_CONTINUE 0x3E   /* Memory Read Continue */
#define ST7796S_WRITE_TEAR_SCAN_LINE 0x44   /* Write Tear Scan Line */
#define ST7796S_READ_SCAN_LINE 0x45         /* Read Scan Line */
#define ST7796S_WRITE_DISPLAY_BRIGHTNESS_VALUE                                 \
    0x51 /* Write Display Brightness Value */
#define ST7796S_READ_DISPLAY_BRIGHTNESS_VALUE                                  \
    0x52 /* Read Display Brightness Value */
#define ST7796S_WRITE_CTRL_DISPLAY_VALUE 0x53 /* Write Ctrl Display Value */
#define ST7796S_READ_CTRL_DISPLAY_VALUE 0x54  /* Read Ctrl Display Value */
#define ST7796S_WRITE_CONTENT_ADAPTIVE_BRIGHTNESS_CTRL_VALUE                   \
    0x55 /* Write Content Adaptive Brightness Control Value */
#define ST7796S_READ_CONTENT_ADAPTIVE_BRIGHTNESS_CTRL_VALUE                    \
    0x56 /* Read Content Adaptive Brightness Control Value */
#define ST7796S_WRITE_CABC_MINIMUM_BRIGHTNESS                                  \
    0x5E /* Write Cabc Minimum Brightness */
#define ST7796S_READ_CABC_MINIMUM_BRIGHTNESS                                   \
    0x5F                                    /* Read Cabc Minimum Brightness */
#define ST7796S_READ_FIRST_CHECKSUM 0xAA    /* Read First Checksum */
#define ST7796S_READ_CONTINUE_CHECKSUM 0xAF /* Read Continue Checksum */
#define ST7796S_READ_ID1 0xDA               /* Read ID1 */
#define ST7796S_READ_ID2 0xDB               /* Read ID2 */
#define ST7796S_READ_ID3 0xDC               /* Read ID3 */
#define ST7796S_INTERFACE_MODE_CTRL 0xB0    /* Interface Mode Control */
#define ST7796S_FR_CTRL_NORMAL_MODE_FULL_COLORS                                \
    0xB1 /* Frame Rate Control (In Normal Mode/full Colors) */
#define ST7796S_FR_CTRL_IDLE_MODE_8_COLORS                                     \
    0xB2 /* Frame Rate Control (In Idle Mode/8 Colors) */
#define ST7796S_FR_CTRL_PARTIAL_MODE_FULL_COLORS                               \
    0xB3 /* Frame Rate Control (In Partial Mode/full Colors) */
#define ST7796S_DISPLAY_INVERSION_CTRL 0xB4 /* Display Inversion Control */
#define ST7796S_BLANKING_PORCH_CTRL 0xB5    /* Blanking Porch Control */
#define ST7796S_DISPLAY_FUNCTION_CTRL 0xB6  /* Display Function Control */
#define ST7796S_ENTRY_MODE_SET 0xB7         /* Entry Mode Set */
#define ST7796S_POWER_CTRL_1 0xC0           /* Power Control 1 */
#define ST7796S_POWER_CTRL_2 0xC1           /* Power Control 2 */
#define ST7796S_POWER_CTRL_3_FOR_NORMAL_MODE                                   \
    0xC2                             /* Power Control 3 (For Normal Mode) */
#define ST7796S_VCOM_CTRL 0xC5       /* Vcom Control */
#define ST7796S_NV_MEMORY_WRITE 0xD0 /* Nv Memory Write */
#define ST7796S_NV_MEMORY_PROTECTION_KEY 0xD1 /* Nv Memory Protection Key */
#define ST7796S_NV_MEMORY_STATUS_READ 0xD2    /* Nv Memory Status Read */
#define ST7796S_READ_ID4 0xD3                 /* Read ID4 */
#define ST7796S_PGAMCTRL 0xE0 /* PGAMCTRL(Positive Gamma Control) */
#define ST7796S_NGAMCTRL 0xE1 /* NGAMCTRL (Negative Gamma Correction) */
#define ST7796S_DIGITAL_GAMMA_CTRL_1 0xE2     /* Digital Gamma Control 1 */
#define ST7796S_DIGITAL_GAMMA_CTRL_2 0xE3     /* Digital Gamma Control 2 */
#define ST7796S_DISPLAY_OUTPUT_CTRL_ADJ 0xE8  /* Display Output Ctrl Adjust */
#define ST7796S_COMMAND_SET_CONTROL 0xF0      /* Command Set Control */
#define ST7796S_SPI_READ_COMMAND_SETTING 0xFB /* Spi Read Command Setting */

/* ST7796S LCD Register Bit Definitions ********************************/

/* ST7796S_INTERFACE_MODE_CTRL, SDA_EN: 0, EPL: 0, DPL: 0, HSPL: 0, VSPL: 0 */

#define ST7796S_INTERFACE_MODE_CONTROL_EPL_SHIFT (0)
#define ST7796S_INTERFACE_MODE_CONTROL_EPL_MASK                                \
    (1 << ST7796S_INTERFACE_MODE_CONTROL_EPL_SHIFT)
#define ST7796S_INTERFACE_MODE_CONTROL_EPL(n)                                  \
    ((n) << ST7796S_INTERFACE_MODE_CONTROL_EPL_SHIFT)
#define ST7796S_INTERFACE_MODE_CONTROL_DPL_SHIFT (1)
#define ST7796S_INTERFACE_MODE_CONTROL_DPL_MASK                                \
    (1 << ST7796S_INTERFACE_MODE_CONTROL_DPL_SHIFT)
#define ST7796S_INTERFACE_MODE_CONTROL_DPL(n)                                  \
    ((n) << ST7796S_INTERFACE_MODE_CONTROL_DPL_SHIFT)
#define ST7796S_INTERFACE_MODE_CONTROL_HSPL_SHIFT (2)
#define ST7796S_INTERFACE_MODE_CONTROL_HSPL_MASK                               \
    (1 << ST7796S_INTERFACE_MODE_CONTROL_HSPL_SHIFT)
#define ST7796S_INTERFACE_MODE_CONTROL_HSPL(n)                                 \
    ((n) << ST7796S_INTERFACE_MODE_CONTROL_HSPL_SHIFT)
#define ST7796S_INTERFACE_MODE_CONTROL_VSPL_SHIFT (3)
#define ST7796S_INTERFACE_MODE_CONTROL_VSPL_MASK                               \
    (1 << ST7796S_INTERFACE_MODE_CONTROL_VSPL_SHIFT)
#define ST7796S_INTERFACE_MODE_CONTROL_VSPL(n)                                 \
    ((n) << ST7796S_INTERFACE_MODE_CONTROL_VSPL_SHIFT)
#define ST7796S_INTERFACE_MODE_CONTROL_SDA_EN_SHIFT (7)
#define ST7796S_INTERFACE_MODE_CONTROL_SDA_EN_MASK                             \
    (1 << ST7796S_INTERFACE_MODE_CONTROL_SDA_EN_SHIFT)
#define ST7796S_INTERFACE_MODE_CONTROL_SDA_EN(n)                               \
    ((n) << ST7796S_INTERFACE_MODE_CONTROL_SDA_EN_SHIFT)

/* ST7796S_INTERFACE_PIXEL_FORMAT, DPI: 0, DBI: 0 */

#define ST7796S_INTERFACE_PIXEL_FORMAT_DBI_SHIFT (0)
#define ST7796S_INTERFACE_PIXEL_FORMAT_DBI_MASK                                \
    (7 << ST7796S_INTERFACE_PIXEL_FORMAT_DBI_SHIFT)
#define ST7796S_INTERFACE_PIXEL_FORMAT_DBI(n)                                  \
    (((n) << ST7796S_INTERFACE_PIXEL_FORMAT_DBI_SHIFT) &                       \
     ST7796S_INTERFACE_PIXEL_FORMAT_DBI_MASK)
#define ST7796S_INTERFACE_PIXEL_FORMAT_DPI_SHIFT (4)
#define ST7796S_INTERFACE_PIXEL_FORMAT_DPI_MASK                                \
    (15 << ST7796S_INTERFACE_PIXEL_FORMAT_DPI_SHIFT)
#define ST7796S_INTERFACE_PIXEL_FORMAT_DPI(n)                                  \
    (((n) << ST7796S_INTERFACE_PIXEL_FORMAT_DPI_SHIFT) &                       \
     ST7796S_INTERFACE_PIXEL_FORMAT_DPI_MASK)

/* ST7796S_POWER_CONTROL_1, DPI: 0, DBI: 0 */

#define ST7796S_POWER_CONTROL_1_VRH1_SHIFT (0)
#define ST7796S_POWER_CONTROL_1_VRH1_MASK                                      \
    (31 << ST7796S_POWER_CONTROL_1_VRH1_SHIFT)
#define ST7796S_POWER_CONTROL_1_VRH1(n)                                        \
    (((n) << ST7796S_POWER_CONTROL_1_VRH1_SHIFT) &                             \
     ST7796S_POWER_CONTROL_1_VRH1_MASK)
#define ST7796S_POWER_CONTROL_1_VRH2_SHIFT (0)
#define ST7796S_POWER_CONTROL_1_VRH2_MASK                                      \
    (31 << ST7796S_POWER_CONTROL_1_VRH2_SHIFT)
#define ST7796S_POWER_CONTROL_1_VRH2(n)                                        \
    (((n) << ST7796S_POWER_CONTROL_1_VRH2_SHIFT) &                             \
     ST7796S_POWER_CONTROL_1_VRH2_MASK)

/* Memory Access control */

#define ST7796S_MEMORY_ACCESS_CONTROL_MH                                       \
    (1 << 2)                                       /* Horizontal refresh order \
                                                    */
#define ST7796S_MEMORY_ACCESS_CONTROL_BGR (1 << 3) /* RGB/BGR order */
#define ST7796S_MEMORY_ACCESS_CONTROL_ML (1 << 4)  /* Vertical refresh order */
#define ST7796S_MEMORY_ACCESS_CONTROL_MV (1 << 5)  /* Row/column exchange */
#define ST7796S_MEMORY_ACCESS_CONTROL_MX (1 << 6)  /* Column address order */
#define ST7796S_MEMORY_ACCESS_CONTROL_MY (1 << 7)  /* Row address order */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct st7796s_lcd_s
{
    /* Interface to control the ST7796S lcd driver
     *
     *  - reset       Switch reset pin of LCD (optional but as your risk).
     *  - select      Select the device (as necessary) before performing any
     *                operations.
     *  - deselect    Deselect the device (as necessary).
     *  - cmddata     Select command (A0 = 0) or data (A0 = 1) mode .
     *  - sendcmd     Send specific command to the LCD driver.
     *  - sendparam   Send specific parameter to the LCD driver.
     *  - recvparam   Receive specific parameter from the LCD driver.
     *  - sendgram    Send pixel data to the LCD drivers gram.
     *  - recvgram    Receive pixel data from the LCD drivers gram.
     *  - backlight   Change the backlight level of the connected display.
     *                In the context of the st7796s that means change the
     *                backlight level of the connected LED driver.
     *                The implementation in detail is part of the platform
     *                specific sub driver.
     *
     */

    void (*reset)(struct st7796s_lcd_s *lcd, const bool on);
    void (*select)(struct st7796s_lcd_s *lcd);
    void (*deselect)(struct st7796s_lcd_s *lcd);
    void (*cmddata)(struct st7796s_lcd_s *lcd, const bool dc);
    int (*sendcmd)(struct st7796s_lcd_s *lcd, const uint8_t cmd);
    int (*sendparam)(struct st7796s_lcd_s *lcd, const uint8_t param);
    int (*recvparam)(struct st7796s_lcd_s *lcd, uint8_t *param);
    int (*recvgram)(struct st7796s_lcd_s *lcd, const uint16_t *wd,
                    uint32_t nwords);
    int (*sendgram)(struct st7796s_lcd_s *lcd, const uint16_t *wd,
                    uint32_t nwords);
    int (*backlight)(struct st7796s_lcd_s *lcd, int level);

    /* mcu interface specific data following */
};

enum st7796s_lcd_rotation_e
{
    PORTRAIT = 0,  /* Default 0° of rotation */
    LANDSCAPE = 1, /* 90° rotation */
    RPORTRAIT = 2, /* 180° rotation reversed portrait */
    RLANDSCAPE = 3 /* 270° rotation reversed landscape */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
    #define EXTERN extern "C"
extern "C"
{
#else
    #define EXTERN extern
#endif

    /****************************************************************************
     * Public Function Prototypes
     ****************************************************************************/

    /****************************************************************************
     * Name:  st7796s_initialize
     *
     * Description:
     *  Initialize the LCD video driver internal structure. Also initialize the
     *  lcd hardware if not done. The control of the LCD driver is depend on the
     *  selected MCU interface and part of the platform specific subdriver (see
     *  config/stm32f429i-disco/src/stm32_st7796s4ws.c)
     *
     * Input Parameters:
     *
     *  lcd - A reference to the platform specific driver instance to control
     *  the st7796s display driver.
     *
     * Returned Value:
     *
     *  On success, this function returns a reference to the LCD driver object
     *  for the specified LCD driver. NULL is returned on any failure.
     *
     ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_PLATFORM_LCD_ST7796S_H */
