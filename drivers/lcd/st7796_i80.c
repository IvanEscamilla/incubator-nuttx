/****************************************************************************
 * drivers/lcd/st7796_i80.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* ST7796 on an 8-bit i80 (8080) bus, as an lcd_dev_s for /dev/lcdN.  The
 * bus (command and strided pixel writes) comes from the board through
 * struct st7796_i80_ops_s.  LCDDEVIO_PUTAREA carries the row stride of the
 * source buffer, so a rectangle of a larger frame buffer is written without
 * copying it.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/video/fb.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7796.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ST7796_SWRESET          0x01
#define ST7796_SLPOUT           0x11
#define ST7796_NORON            0x13
#define ST7796_INVOFF           0x20
#define ST7796_INVON            0x21
#define ST7796_DISPOFF          0x28
#define ST7796_DISPON           0x29
#define ST7796_CASET            0x2a
#define ST7796_RASET            0x2b
#define ST7796_RAMWR            0x2c
#define ST7796_TEON             0x35
#define ST7796_MADCTL           0x36
#define ST7796_PIXFMT           0x3a
#define ST7796_INVCTR           0xb4
#define ST7796_DFC              0xb6
#define ST7796_PWCTRL2          0xc1
#define ST7796_PWCTRL3          0xc2
#define ST7796_VCOM             0xc5
#define ST7796_GAMMAPOS         0xe0
#define ST7796_GAMMANEG         0xe1
#define ST7796_CSCON            0xf0

#define ST7796_MADCTL_MY        0x80
#define ST7796_MADCTL_MX        0x40
#define ST7796_MADCTL_MV        0x20
#define ST7796_MADCTL_BGR       0x08

/* Effective resolution from the raw panel size and the orientation */

#if defined(CONFIG_LCD_ST7796_LANDSCAPE) || \
    defined(CONFIG_LCD_ST7796_RLANDSCAPE)
#  define ST7796_XRES  CONFIG_LCD_ST7796_YRES
#  define ST7796_YRES  CONFIG_LCD_ST7796_XRES
#else
#  define ST7796_XRES  CONFIG_LCD_ST7796_XRES
#  define ST7796_YRES  CONFIG_LCD_ST7796_YRES
#endif

/* Base MADCTL: orientation, optional additional 180 degree flip and the
 * colour order.
 */

#if defined(CONFIG_LCD_ST7796_LANDSCAPE)
#  define ST7796_MADCTL_ORIENT  (ST7796_MADCTL_MV)
#elif defined(CONFIG_LCD_ST7796_RLANDSCAPE)
#  define ST7796_MADCTL_ORIENT  (ST7796_MADCTL_MY | ST7796_MADCTL_MX | \
                                 ST7796_MADCTL_MV)
#elif defined(CONFIG_LCD_ST7796_RPORTRAIT)
#  define ST7796_MADCTL_ORIENT  (ST7796_MADCTL_MY)
#else
#  define ST7796_MADCTL_ORIENT  (ST7796_MADCTL_MX)
#endif

#if CONFIG_LCD_ST7796_ROTATION == 180
#  define ST7796_MADCTL_ROT     (ST7796_MADCTL_MX | ST7796_MADCTL_MY)
#else
#  define ST7796_MADCTL_ROT     0
#endif

#ifdef CONFIG_LCD_ST7796_BGR
#  define ST7796_MADCTL_COLOR   ST7796_MADCTL_BGR
#else
#  define ST7796_MADCTL_COLOR   0
#endif

#define ST7796_MADCTL_VALUE     (ST7796_MADCTL_ORIENT ^ ST7796_MADCTL_ROT ^ \
                                 ST7796_MADCTL_COLOR)

#if CONFIG_LCD_ST7796_BPP != 16
#  error "The ST7796 i80 driver supports RGB565 only"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct st7796_cmd_s
{
  uint8_t        cmd;       /* Command byte */
  FAR const uint8_t *data;  /* Parameter data (NULL if no params) */
  uint8_t        len;       /* Number of parameter bytes */
  uint16_t       delay_ms;  /* Delay after the command */
};

struct st7796_i80_dev_s
{
  struct lcd_dev_s dev;     /* Must be first */
  FAR const struct st7796_i80_ops_s *ops;
  mutex_t lock;
  bool power;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_cscon1[] =
{
  0xc3
};

static const uint8_t g_cscon2[] =
{
  0x96
};

static const uint8_t g_pixfmt[] =
{
  0x55
};

static const uint8_t g_invctr[] =
{
  0x01
};

static const uint8_t g_dfc[] =
{
  0x80, 0x02, 0x3b
};

static const uint8_t g_cmd_e8[] =
{
  0x40, 0x8a, 0x00, 0x00, 0x29, 0x19, 0xa5, 0x33
};

static const uint8_t g_pwctrl2[] =
{
  0x06
};

static const uint8_t g_pwctrl3[] =
{
  0xa7
};

static const uint8_t g_vcom[] =
{
  0x18
};

static const uint8_t g_gammapos[] =
{
  0xf0, 0x09, 0x0b, 0x06, 0x04, 0x15, 0x2f, 0x54, 0x42, 0x3c, 0x17, 0x14,
  0x18, 0x1b
};

static const uint8_t g_gammaneg[] =
{
  0xf0, 0x09, 0x0b, 0x06, 0x04, 0x03, 0x2d, 0x43, 0x42, 0x3b, 0x16, 0x14,
  0x17, 0x1b
};

static const uint8_t g_cscon3[] =
{
  0x3c
};

static const uint8_t g_cscon4[] =
{
  0x69
};

static const uint8_t g_madctl[] =
{
  ST7796_MADCTL_VALUE
};

/* TEON mode 0: the TE pin pulses in V-blank only */

static const uint8_t g_teon[] =
{
  0x00
};

static const struct st7796_cmd_s g_init_seq[] =
{
  { ST7796_SWRESET, NULL,      0,  150 },
  { ST7796_SLPOUT,  NULL,      0,  120 },
  { ST7796_CSCON,   g_cscon1,  1,  0   },
  { ST7796_CSCON,   g_cscon2,  1,  0   },
  { ST7796_MADCTL,  g_madctl,  1,  0   },
  { ST7796_PIXFMT,  g_pixfmt,  1,  0   },
  { ST7796_INVCTR,  g_invctr,  1,  0   },
  { ST7796_DFC,     g_dfc,     3,  0   },
  { 0xe8,           g_cmd_e8,  8,  0   },
  { ST7796_PWCTRL2, g_pwctrl2, 1,  0   },
  { ST7796_PWCTRL3, g_pwctrl3, 1,  0   },
  { ST7796_VCOM,    g_vcom,    1,  0   },
  { ST7796_GAMMAPOS, g_gammapos, 14, 0 },
  { ST7796_GAMMANEG, g_gammaneg, 14, 0 },
  { ST7796_CSCON,   g_cscon3,  1,  0   },
  { ST7796_CSCON,   g_cscon4,  1,  0   },
#ifdef CONFIG_LCD_ST7796_INVCOLOR
  { ST7796_INVON,   NULL,      0,  10  },
#else
  { ST7796_INVOFF,  NULL,      0,  10  },
#endif
#ifdef CONFIG_LCD_ST7796_TE
  { ST7796_TEON,    g_teon,    1,  0   },
#endif
  { ST7796_NORON,   NULL,      0,  10  },
  { ST7796_DISPON,  NULL,      0,  120 },
};

static struct st7796_i80_dev_s g_st7796_i80;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st7796_i80_setwindow
 *
 * Description:
 *   Set the write window (CASET, RASET).
 *
 ****************************************************************************/

static int st7796_i80_setwindow(FAR struct st7796_i80_dev_s *priv,
                                fb_coord_t row_start, fb_coord_t row_end,
                                fb_coord_t col_start, fb_coord_t col_end)
{
  uint8_t data[4];
  int ret;

  data[0] = (col_start >> 8) & 0xff;
  data[1] = col_start & 0xff;
  data[2] = (col_end >> 8) & 0xff;
  data[3] = col_end & 0xff;
  ret = priv->ops->cmd(ST7796_CASET, data, 4);
  if (ret < 0)
    {
      return ret;
    }

  data[0] = (row_start >> 8) & 0xff;
  data[1] = row_start & 0xff;
  data[2] = (row_end >> 8) & 0xff;
  data[3] = row_end & 0xff;
  return priv->ops->cmd(ST7796_RASET, data, 4);
}

/****************************************************************************
 * Name: st7796_i80_putarea
 *
 * Description:
 *   Write a rectangle.  buffer points at its first pixel and stride is the
 *   distance in bytes between its rows, so it can lie inside a bigger frame.
 *
 ****************************************************************************/

static int st7796_i80_putarea(FAR struct lcd_dev_s *dev,
                              fb_coord_t row_start, fb_coord_t row_end,
                              fb_coord_t col_start, fb_coord_t col_end,
                              FAR const uint8_t *buffer, fb_coord_t stride)
{
  FAR struct st7796_i80_dev_s *priv = (FAR struct st7796_i80_dev_s *)dev;
  size_t row_bytes;
  int ret;

  if (row_start > row_end || col_start > col_end ||
      row_end >= ST7796_YRES || col_end >= ST7796_XRES || buffer == NULL)
    {
      return -EINVAL;
    }

  row_bytes = (size_t)(col_end - col_start + 1) * 2;
  if (stride < row_bytes)
    {
      stride = row_bytes;
    }

  nxmutex_lock(&priv->lock);
  ret = st7796_i80_setwindow(priv, row_start, row_end, col_start, col_end);
  if (ret >= 0)
    {
      ret = priv->ops->write(ST7796_RAMWR, buffer, row_bytes,
                             row_end - row_start + 1, stride);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int st7796_i80_putrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                             fb_coord_t col, FAR const uint8_t *buffer,
                             size_t npixels)
{
  return st7796_i80_putarea(dev, row, row, col, col + npixels - 1, buffer,
                            npixels * 2);
}

static int st7796_i80_getrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                             fb_coord_t col, FAR uint8_t *buffer,
                             size_t npixels)
{
  return -ENOSYS;
}

static int st7796_i80_getarea(FAR struct lcd_dev_s *dev,
                              fb_coord_t row_start, fb_coord_t row_end,
                              fb_coord_t col_start, fb_coord_t col_end,
                              FAR uint8_t *buffer, fb_coord_t stride)
{
  return -ENOSYS;
}

static int st7796_i80_getvideoinfo(FAR struct lcd_dev_s *dev,
                                   FAR struct fb_videoinfo_s *vinfo)
{
  vinfo->fmt     = FB_FMT_RGB16_565;
  vinfo->xres    = ST7796_XRES;
  vinfo->yres    = ST7796_YRES;
  vinfo->nplanes = 1;
  return OK;
}

static int st7796_i80_getplaneinfo(FAR struct lcd_dev_s *dev,
                                   unsigned int planeno,
                                   FAR struct lcd_planeinfo_s *pinfo)
{
  if (planeno != 0)
    {
      return -EINVAL;
    }

  pinfo->putrun  = st7796_i80_putrun;
  pinfo->putarea = st7796_i80_putarea;
  pinfo->getrun  = st7796_i80_getrun;
  pinfo->getarea = st7796_i80_getarea;
  pinfo->buffer  = NULL;
  pinfo->bpp     = 16;
  pinfo->dev     = dev;
  return OK;
}

static int st7796_i80_getpower(FAR struct lcd_dev_s *dev)
{
  return ((FAR struct st7796_i80_dev_s *)dev)->power ? 1 : 0;
}

static int st7796_i80_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct st7796_i80_dev_s *priv = (FAR struct st7796_i80_dev_s *)dev;
  bool on = power > 0;
  int ret;

  nxmutex_lock(&priv->lock);
  ret = priv->ops->cmd(on ? ST7796_DISPON : ST7796_DISPOFF, NULL, 0);
  nxmutex_unlock(&priv->lock);
  if (ret >= 0)
    {
      priv->power = on;
      if (priv->ops->power != NULL)
        {
          priv->ops->power(on);
        }
    }

  return ret;
}

static int st7796_i80_getcontrast(FAR struct lcd_dev_s *dev)
{
  return -ENOSYS;
}

static int st7796_i80_setcontrast(FAR struct lcd_dev_s *dev,
                                  unsigned int contrast)
{
  return -ENOSYS;
}

static int st7796_i80_setframerate(FAR struct lcd_dev_s *dev, int rate)
{
  return -ENOSYS;
}

static int st7796_i80_getframerate(FAR struct lcd_dev_s *dev)
{
  return -ENOSYS;
}

static int st7796_i80_getareaalign(FAR struct lcd_dev_s *dev,
                                   FAR struct lcddev_area_align_s *align)
{
  align->row_start_align = 1;
  align->height_align    = 1;
  align->col_start_align = 1;
  align->width_align     = 1;
  align->buf_align       = 2;
  return OK;
}

/****************************************************************************
 * Name: st7796_i80_ioctl
 *
 * Description:
 *   The ST7796IOC_* commands (TE wait, statistics, clock, mode) are handled
 *   by the bus provided by the board.
 *
 ****************************************************************************/

static int st7796_i80_ioctl(FAR struct lcd_dev_s *dev, int cmd,
                            unsigned long arg)
{
  FAR struct st7796_i80_dev_s *priv = (FAR struct st7796_i80_dev_s *)dev;

  if (priv->ops->ioctl == NULL)
    {
      return -ENOTTY;
    }

  return priv->ops->ioctl(cmd, arg);
}

/****************************************************************************
 * Name: st7796_i80_initdisplay
 ****************************************************************************/

static int st7796_i80_initdisplay(FAR struct st7796_i80_dev_s *priv)
{
  int i;
  int ret;

  for (i = 0; i < nitems(g_init_seq); i++)
    {
      ret = priv->ops->cmd(g_init_seq[i].cmd, g_init_seq[i].data,
                           g_init_seq[i].len);
      if (ret < 0)
        {
          lcderr("ERROR: ST7796 command 0x%02x failed: %d\n",
                 g_init_seq[i].cmd, ret);
          return ret;
        }

      if (g_init_seq[i].delay_ms != 0)
        {
          nxsig_usleep(g_init_seq[i].delay_ms * 1000);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st7796_i80_initialize
 ****************************************************************************/

FAR struct lcd_dev_s *st7796_i80_initialize(
                          FAR const struct st7796_i80_ops_s *ops)
{
  FAR struct st7796_i80_dev_s *priv = &g_st7796_i80;

  DEBUGASSERT(ops != NULL && ops->cmd != NULL && ops->write != NULL);

  priv->ops                  = ops;
  nxmutex_init(&priv->lock);

  priv->dev.getvideoinfo     = st7796_i80_getvideoinfo;
  priv->dev.getplaneinfo     = st7796_i80_getplaneinfo;
  priv->dev.getpower         = st7796_i80_getpower;
  priv->dev.setpower         = st7796_i80_setpower;
  priv->dev.getcontrast      = st7796_i80_getcontrast;
  priv->dev.setcontrast      = st7796_i80_setcontrast;
  priv->dev.setframerate     = st7796_i80_setframerate;
  priv->dev.getframerate     = st7796_i80_getframerate;
  priv->dev.getareaalign     = st7796_i80_getareaalign;
  priv->dev.ioctl            = st7796_i80_ioctl;

  if (st7796_i80_initdisplay(priv) < 0)
    {
      return NULL;
    }

  priv->power = true;
  if (ops->power != NULL)
    {
      ops->power(true);
    }

  lcdinfo("ST7796 i80: %dx%d MADCTL=0x%02x\n", ST7796_XRES, ST7796_YRES,
          ST7796_MADCTL_VALUE);
  return &priv->dev;
}
