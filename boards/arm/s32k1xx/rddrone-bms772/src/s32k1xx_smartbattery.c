/****************************************************************************
 * boards/arm/s32k1xx/rddrone-bms772/src/s32k1xx_smartbattery.c
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

#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>

#include <nuttx/i2c/i2c_slave.h>

#include "s32k1xx_lpi2c_slave.h"

#include "../../drivers/smart_battery/simple_sbs.h"

#if defined(CONFIG_S32K1XX_LPI2C) && defined(CONFIG_I2C_SLAVE) && defined(CONFIG_SBS_DRIVER)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

int s32k1xx_smartbattery_update_example(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Example of a smart battery data structure that can be written to the SBS
 * driver.  This same data can then be requested from the SBS I2C slave.
 */

static const char manufacture_name[] = "NXP";
static const char device_name[]      = "BMS772";
static const char device_chemistry[] = "LiP";

static const uint8_t manufacturer_data[] =
{
  0x0a,
  0x0b,
  0x0c,
  0x0d,
  0x0e,
  0x0f,
};

static struct sbs_data_s sbs_data =
{
  .temperature              = 2932,  /* 0.1  K */
  .voltage                  = 16000, /* 1.0 mV */
  .current                  = 10000, /* 1.0 mA */
  .average_current          = 5000,  /* 1.0 mA */
  .max_error                = 10,    /* 1.0  % */
  .relative_state_of_charge = 50,    /* 1.0  % */
  .absolute_state_of_charge = 60,    /* 1.0  % */
  .remaining_capacity       = 2000,  /* 1.0 mAh (or 10 mWh?) */
  .full_charge_capacity     = 4200,  /* 1.0 mAh (or 10 mWh?) */
  .run_time_to_empty        = 10,    /* 1.0  min */
  .average_time_to_empty    = 12,    /* 1.0  min */

  .cycle_count              = 20,    /* 1.0  cycle */
  .design_capacity          = 4200,  /* 1.0 mAh (or 10 mWh?) */
  .design_voltage           = 16800, /* 1.0 mV */

  .manufacture_date         = ((1996 - 1980) * 512 + 12 * 32 + 24), /* (year - 1980) * 512 + month * 32 + day */
  .serial_number            = 777,
  .manufacturer_name        = manufacture_name,
  .device_name              = device_name,
  .device_chemistry         = device_chemistry,
  .manufacturer_data        = manufacturer_data,
  .manufacturer_data_length = 6,

  .cell1_voltage            = 4000,  /* 1.0 mV */
  .cell2_voltage            = 4000,  /* 1.0 mV */
  .cell3_voltage            = 4000,  /* 1.0 mV */
  .cell4_voltage            = 4000,  /* 1.0 mV */
  .cell5_voltage            = 0,     /* 1.0 mV */
  .cell6_voltage            = 0,     /* 1.0 mV */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_smartbattery_update_example
 *
 * Description:
 *   Updates the battery data that is known by the Simple SBS driver, which
 *   provides the appropriate data to the I2C Slave when requested.
 *
 *   This is an example that shows how to open the SBS device and write a
 *   struct containing the latest battery data.  In a continuously running
 *   application the file can be kept open, while data is being written to it
 *   at regular intervals.  This can be handled by a loop or a separate task.
 *
 * Returned Value:
 *   0 if the SBS data was successfully updated; -1 on any failure.
 *
 ****************************************************************************/

int s32k1xx_smartbattery_update_example(void)
{
  int fd;
  int ret;

  /* Open the Simple SBS driver that we registered at /dev/sbs0.  This only
   * needs to be done once, there is no need to close the file after every
   * write operation.  You can keep it open indefinitely.
   */

  fd = open("/dev/sbs0", O_WRONLY);
  if (fd == ERROR)
    {
      /* Something went wrong.  You could try to handle the error here, pass
       * it on to the calling function, or just ignore it.
       */

      return -1;
    }

  /* Write a prepared sbs_data_s struct to the SBS driver.  It needs to be
   * converted to a constant character buffer and the buffer length has to be
   * equal to the size of the sbs_dat_s struct.
   *
   * Note that this write operation can be performed as often as you like.
   * The SBS driver will always provide the most recent data that it received
   * when it receives a request for battery data.
   */

  ret = write(fd, (const char *)&sbs_data, sizeof(struct sbs_data_s));
  if (ret != sizeof(struct sbs_data_s))
    {
      /* Something went wrong.  You could try to handle the error here, pass
       * it on to the calling function, or just ignore it.
       */

      return -1;
    }

  /* Close the driver.  You only need to do this before a thread or the whole
   * application exits, so in practice you can keep it open indefinitely.
   */

  ret = close(fd);
  if (ret < 0)
    {
      /* Something went wrong.  You could try to handle the error here, pass
       * it on to the calling function, or just ignore it.
       */

      return -1;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_smartbattery_initialize
 *
 * Description:
 *   Initialize the I2C Slave and Simple SBS drivers.  After this is done the
 *   user can provide battery data to the SBS driver, which will then be
 *   available if requested by a I2C/SMBus bus master.
 *
 * Returned Value:
 *   OK if the SBS driver was successfully initialized; A negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

int s32k1xx_smartbattery_initialize(void)
{
  FAR struct i2c_slave_s *i2c_slave;
  int ret;

  /* Initialize I2C slave device */

  i2c_slave = s32k1xx_i2cbus_slave_initialize(0);
  if (i2c_slave == NULL)
    {
      return -ENODEV;
    }

  /* Initialize a new "Simple SBS" device as /dev/sbs0 */

  ret = sbs_initialize(0, i2c_slave);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialization is done.  You only need to initialize once and there is
   * currently no method to uninitialize the SBS device (and you should not
   * need it).  You can now open the /dev/sbs0 device and write data to it.
   * The function called below provides an example on how this can be done.
   */

  s32k1xx_smartbattery_update_example();

  return OK;
}

#endif /* CONFIG_S32K1XX_LPI2C && CONFIG_I2C_SLAVE && CONFIG_SBS_DRIVER */
