/****************************************************************************
 * boards/arm/s32k1xx/rddrone-bms772/src/s32k1xx_ssd1306.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/pm.h>
#include <sys/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_LCD_MAXPOWER
#  define CONFIG_LCD_MAXPOWER 1
#endif

#define I2C_PATH    "/dev/i2c0"

#if defined(CONFIG_PM)
#ifndef PM_IDLE_DOMAIN
#  define PM_IDLE_DOMAIN      0 /* Revisit */
#endif
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int dowmin,
                         enum pm_state_e pmstate);
static int  up_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);

static bool set_not_get_pm_enable(bool set_not_get, bool new_value);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

FAR struct i2c_master_s *g_i2c;
FAR struct lcd_dev_s    *g_lcddev;

#ifdef CONFIG_PM
static  struct pm_callback_s g_ssd1306_pmcb =
{
  .notify       = up_pm_notify,
  .prepare      = up_pm_prepare,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  /* Initialize I2C */

  // g_i2c = s32k1xx_i2cbus_initialize(0);
  // if (!g_i2c)
  //   {
  //     lcderr("ERROR: Failed to initialize I2C port %d\n", 0);
  //     return -ENODEV;
  //   }

  return OK;
}

int board_lcd_initialize_I2C(FAR struct i2c_master_s * i2cInit)
{
  int ret, fd;
  uint8_t i2c_buffer[2] = {
    0, 0
  };

  struct i2c_msg_s scan_message = {
    .frequency  = CONFIG_SSD1306_I2CFREQ,
    .addr       = CONFIG_SSD1306_I2CADDR,
    .flags      = I2C_M_READ,
    .buffer     = i2c_buffer,
    .length     = 2
  };

  struct i2c_transfer_s i2c_transfer;

  /* set the message count to 1 */

  i2c_transfer.msgc = 1;

  /* Make the i2C tranfer to test display presence */

  i2c_transfer.msgv = (struct i2c_msg_s *)&scan_message;

  /* Initialize I2C */

  g_i2c = i2cInit;

  /* Check for errors */

  if (!g_i2c)
    {
      lcderr("ERROR: Failed to initialize I2C port %d\n", 0);
      return -ENODEV;
    }

  /* Open the i2c device */

  fd = open(I2C_PATH, O_RDONLY);

  /* Check for errors */

  if (fd < 0)
    {
      /* Output to the user */

      lcderr("i2c ERROR: Can't open i2c device, error: %d\n", fd);

      return -ENODEV;
    }

  /* Try to do the i2C transfer to check if display is available */

  ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&i2c_transfer);

  /* Check for errors */

  if (ret)
    {
      lcderr("ERROR: Can't do I2C to display\n");
    }
#ifdef CONFIG_PM
    /* Only register PM part if display is there */
    else
    {
      /* Register to receive power management callbacks */
  
      ret = pm_register(&g_ssd1306_pmcb);
      DEBUGASSERT(ret == OK);
    }
#endif

  /* Close the I2C device */

  close(fd);

  /* Return 0 if succeeded, errror otherwise */

  return ret;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int devno)
{
  
  lcdinfo("Binding I2C port %d to OLED %d\n", 0, devno);

  /* Bind the I2C port to the OLED */

  g_lcddev = ssd1306_initialize(g_i2c, NULL, devno);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind I2C port 1 to OLED %d: %d\n", devno);
    }
  else
    {
      lcdinfo("Bound I2C port %d to OLED %d\n", 0, devno);

      /* And turn the OLED on */

      g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);
      return g_lcddev;
    }

  return NULL;
}

/****************************************************************************
 * Name: board_lcd_setpower
 ****************************************************************************/

int board_lcd_setpower(bool power)
{
  int ret;

  /* Check if the LCD needs to be enabled or not */

  if (power)
    {
      ret = g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);
    }
  else
    {
      ret = g_lcddev->setpower(g_lcddev, 0);
    }

    /* Return */

    return ret;
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* Turn off the power to uninitialize */

  g_lcddev->setpower(g_lcddev, 0);
}

/****************************************************************************
 * Name: up_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   None - The driver already agreed to transition to the low power
 *   consumption state when when it returned OK to the prepare() call.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  /* check if the transition is from the IDLE domain to the NORMAL domain */

  /* or the mode is already done */

  /* Check if the pm should be done */

  if (set_not_get_pm_enable(false, true))
    {
      if (((pm_querystate(PM_IDLE_DOMAIN) == PM_IDLE) &&
        (pmstate == PM_NORMAL)) ||
        (((pm_querystate(PM_IDLE_DOMAIN) == pmstate))))
        {
          /* return */

          return;
        }

      /* Check which PM it is */

      switch (pmstate)
      {
        /* In case it changed to the RUN mode */

        case PM_NORMAL:
        {
          /* Enable the LCD again */

          if (board_lcd_setpower(true))
            {
              /* If failed, unregister this PM */

              lcderr("Failed to setpower(true)");

              /* Make sure you disable the pm from now */

              set_not_get_pm_enable(true, false);
            }
        }
        break;

        default:
        {
          /* don't do anything, just return OK */
        }
        break;
      }
    }

    /* Return */

    return;
}
#endif

/****************************************************************************
 * Name: up_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   Zero - (OK) means the event was successfully processed and that the
 *          driver is prepared for the PM state change.
 *
 *   Non-zero - means that the driver is not prepared to perform the tasks
 *              needed achieve this power setting and will cause the state
 *              change to be aborted. NOTE: The prepare() method will also
 *              be called when reverting from lower back to higher power
 *              consumption modes (say because another driver refused a
 *              lower power state change). Drivers are not permitted to
 *              return non-zero values when reverting back to higher power
 *              consumption modes!
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int up_pm_prepare(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  int ret = OK;

  /* Logic to prepare for a reduced power state goes here. */

  /* Check if the pm should be done */

  if (set_not_get_pm_enable(false, true))
    {
      /* check if the transition to the mode is already done */

      if (pm_querystate(PM_IDLE_DOMAIN) == pmstate)
        {
          /* return */

          return ret;
        }

      /* check which PM it is */

      switch (pmstate)
      {
        /* in case it needs to prepare for VLPR mode */

        case PM_STANDBY:
        {
          /* Logic for PM_STANDBY goes here */

          /* Disable the LCD */

          ret = board_lcd_setpower(false);

          /* Check for errors */

          if (ret)
            {
              /* If failed, unregister this PM */

              lcderr("Failed to setpower(true)");

              /* Make sure you disable the pm from now */

              ret = (int)set_not_get_pm_enable(true, false);
            }
        }
        break;

        /* in case it needs to prepare for VLPR mode */

        case PM_SLEEP:
        {
          /* Logic for PM_STANDBY goes here */

          /* Disable the LCD */

          ret = board_lcd_setpower(false);

          /* Check for errors */

          if (ret)
            {
              /* If failed, unregister this PM */

              lcderr("Failed to setpower(true)");

              /* Make sure you disable the pm from now */

              ret = (int)set_not_get_pm_enable(true, false);
            }
        }
        break;

        default:
        {
          /* don't do anything, just return OK */
        }
        break;
      }
    }

  /* Return OK */

  return ret;
}

static bool set_not_get_pm_enable(bool set_not_get, bool new_value)
{
  static bool pm_enable = true;

  /* Check if the variable should be set */

  if (set_not_get)
    {
      /* Set the static value */

      pm_enable = new_value;
    }

  /* Return the current value */

  return pm_enable;
}

#endif
