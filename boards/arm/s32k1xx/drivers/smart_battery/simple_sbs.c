/****************************************************************************
 * boards/arm/s32k1xx/drivers/smart_battery/simple_sbs.c
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_slave.h>

#include "simple_sbs.h"

#ifdef CONFIG_SBS_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros for splitting a 16-bit unsigned integer into two bytes */

#define LOBYTE(n) ((uint8_t) ((n) & 0x00ff))
#define HIBYTE(n) ((uint8_t) (((n) & 0xff00) >> 8))

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Private data of the SBS device */

struct sbs_dev_s
{
  FAR struct i2c_slave_s *i2c_slave_dev; /* Associated I2C slave device */
  FAR struct sbs_data_s *data;           /* Most recent battery data */

  uint8_t read_buffer[3];   /* Pre-allocated read buffer */
  uint8_t write_buffer[16]; /* Pre-allocated write buffer */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  uint8_t sbs_refs; /* Reference count */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sbs_open(FAR struct file *filep);
static int sbs_close(FAR struct file *filep);
static ssize_t sbs_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t sbs_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Valid operations that can be performed on the the SBS character device */

static const struct file_operations sbs_fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  sbs_open,  /* open */
  sbs_close, /* close */
#else
  NULL,      /* open */
  NULL,      /* close */
#endif
  sbs_read,  /* read */
  sbs_write, /* write */
  NULL,      /* seek */
  NULL,      /* ioctl */
  NULL,      /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL,      /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
/****************************************************************************
 * Name: sbs_open
 *
 * Description: Open the character device.
 *
 * Input Parameters:
 *   filep - Instance of file struct
 *
 * Returned Value:
 *   OK if the SBS character device was successfully opened; A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static int sbs_open(FAR struct file *filep)
{
  FAR struct sbs_dev_s *dev;

  /* Retrieve the sbs_dev_s struct */

  DEBUGASSERT(filep && filep->f_inode && filep->f_inode->i_private);
  dev = (FAR struct sbs_dev_s *)filep->f_inode->i_private;

  /* Increase the open reference count */

  dev->sbs_refs++;
  DEBUGASSERT(dev->sbs_refs > 0);

  return OK;
}

/****************************************************************************
 * Name: sbs_close
 *
 * Description:
 *   Close the character device.
 *
 * Input Parameters:
 *   filep - Instance of file struct
 *
 * Returned Value:
 *   OK if the SBS character device was successfully closed; A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static int sbs_close(FAR struct file *filep)
{
  FAR struct sbs_dev_s *dev;

  /* Retrieve the sbs_dev_s struct */

  DEBUGASSERT(filep && filep->f_inode && filep->f_inode->i_private);
  dev = (FAR struct sbs_dev_s *)filep->f_inode->i_private;

  /* Decrease the open reference count */

  DEBUGASSERT(dev->sbs_refs > 0);
  dev->sbs_refs--;

  return OK;
}
#endif /* CONFIG_DISABLE_PSEUDOFS_OPERATIONS */

/****************************************************************************
 * Name: sbs_read
 *
 * Description:
 *   Reads the battery data that is currently known by the SBS driver into a
 *   sbs_data_s struct that needs to be converted to a character buffer.
 *
 * Input Parameters:
 *   filep  - Instance of file struct
 *   buffer - Pointer to an empty sbs_data_s struct (cast to a char *)
 *            which will be filled with the most recent battery data that is
 *            available in the SBS driver.
 *   buflen - Size of the provided sbs_data_s struct
 *
 * Returned Value:
 *   On success, buflen is returned to indicate that all data has been
 *   copied.  Zero will be returned if the provided buffer length is not
 *   sufficient for a sbs_data_s struct.
 *
 ****************************************************************************/

static ssize_t sbs_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  irqstate_t flags;
  struct sbs_dev_s *dev;
  struct sbs_data_s *read_data;

  /* Make sure that the read process (i.e. copying data from the SBS
   * driver) cannot be interrupted.
   */

  flags = enter_critical_section();

  if (buflen < sizeof(struct sbs_data_s))
    {
      /* Something went wrong. The provided buffer length is not sufficient
       * for a sbs_data_s struct.  Return zero to indicate that nothing was
       * read.
       */

      return 0;
    }

  /* Retrieve the SBS device struct and the data struct that holds the new
   * data that should be copied to the SBS driver.
   */

  DEBUGASSERT(filep && filep->f_inode && filep->f_inode->i_private);
  dev = (FAR struct sbs_dev_s *)filep->f_inode->i_private;

  DEBUGASSERT(buffer);
  read_data = (struct sbs_data_s *)buffer;

  /* Copy the new data into the SBS device struct */

  read_data->temperature              = dev->data->temperature;
  read_data->voltage                  = dev->data->voltage;
  read_data->current                  = dev->data->current;
  read_data->average_current          = dev->data->average_current;
  read_data->max_error                = dev->data->max_error;
  read_data->relative_state_of_charge = dev->data->relative_state_of_charge;
  read_data->absolute_state_of_charge = dev->data->absolute_state_of_charge;
  read_data->remaining_capacity       = dev->data->remaining_capacity;
  read_data->full_charge_capacity     = dev->data->full_charge_capacity;
  read_data->run_time_to_empty        = dev->data->run_time_to_empty;
  read_data->average_time_to_empty    = dev->data->average_time_to_empty;

  read_data->cycle_count              = dev->data->cycle_count;
  read_data->design_capacity          = dev->data->design_capacity;
  read_data->design_voltage           = dev->data->design_voltage;
  read_data->manufacture_date         = dev->data->manufacture_date;
  read_data->serial_number            = dev->data->serial_number;
  read_data->manufacturer_name        = dev->data->manufacturer_name;
  read_data->device_name              = dev->data->device_name;
  read_data->device_chemistry         = dev->data->device_chemistry;
  read_data->manufacturer_data        = dev->data->manufacturer_data;
  read_data->manufacturer_data_length = dev->data->manufacturer_data_length;

  read_data->cell1_voltage            = dev->data->cell1_voltage;
  read_data->cell2_voltage            = dev->data->cell2_voltage;
  read_data->cell3_voltage            = dev->data->cell3_voltage;
  read_data->cell4_voltage            = dev->data->cell4_voltage;
  read_data->cell5_voltage            = dev->data->cell5_voltage;
  read_data->cell6_voltage            = dev->data->cell6_voltage;

  leave_critical_section(flags);

  return buflen;
}

/****************************************************************************
 * Name: sbs_write
 *
 * Description:
 *   Updates the battery data of the SBS driver.  The data is contained in a
 *   sbs_data_s struct that needs to be converted to a character buffer.
 *   This data is then copied into the private data structure of the SBS
 *   driver and used to prepare a write buffer for the I2C slave when a valid
 *   request is received on the I2C bus.
 *
 * Input Parameters:
 *   filep  - Instance of file struct
 *   buffer - Pointer to a sbs_data_s struct (cast to a const char *)
 *            containing updated battery data.
 *   buflen - Size of the provided sbs_data_s struct
 *
 * Returned Value:
 *   On success, buflen is returned to indicate that all data has been
 *   copied.  Zero will be returned if the provided buffer length is not
 *   sufficient for a sbs_data_s struct.
 *
 ****************************************************************************/

static ssize_t sbs_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  irqstate_t flags;
  struct sbs_dev_s *dev;
  struct sbs_data_s *new_data;

  /* Make sure that the write process (i.e. copying new data to the SBS
   * driver) cannot be interrupted.
   */

  flags = enter_critical_section();

  if (buflen < sizeof(struct sbs_data_s))
    {
      /* Something went wrong. The provided buffer length is not sufficient
       * for a sbs_data_s struct.  Return zero to indicate that nothing was
       * written.
       */

      return 0;
    }

  /* Retrieve the SBS device struct and the data struct that holds the new
   * data that should be copied to the SBS driver.
   */

  DEBUGASSERT(filep && filep->f_inode && filep->f_inode->i_private);
  dev = (FAR struct sbs_dev_s *)filep->f_inode->i_private;

  DEBUGASSERT(buffer);
  new_data = (struct sbs_data_s *)buffer;

  /* Copy the new data into the SBS device struct */

  dev->data->temperature              = new_data->temperature;
  dev->data->voltage                  = new_data->voltage;
  dev->data->current                  = new_data->current;
  dev->data->average_current          = new_data->average_current;
  dev->data->max_error                = new_data->max_error;
  dev->data->relative_state_of_charge = new_data->relative_state_of_charge;
  dev->data->absolute_state_of_charge = new_data->absolute_state_of_charge;
  dev->data->remaining_capacity       = new_data->remaining_capacity;
  dev->data->full_charge_capacity     = new_data->full_charge_capacity;
  dev->data->run_time_to_empty        = new_data->run_time_to_empty;
  dev->data->average_time_to_empty    = new_data->average_time_to_empty;

  dev->data->cycle_count              = new_data->cycle_count;
  dev->data->design_capacity          = new_data->design_capacity;
  dev->data->design_voltage           = new_data->design_voltage;
  dev->data->manufacture_date         = new_data->manufacture_date;
  dev->data->serial_number            = new_data->serial_number;
  dev->data->manufacturer_name        = new_data->manufacturer_name;
  dev->data->device_name              = new_data->device_name;
  dev->data->device_chemistry         = new_data->device_chemistry;
  dev->data->manufacturer_data        = new_data->manufacturer_data;
  dev->data->manufacturer_data_length = new_data->manufacturer_data_length;

  dev->data->cell1_voltage            = new_data->cell1_voltage;
  dev->data->cell2_voltage            = new_data->cell2_voltage;
  dev->data->cell3_voltage            = new_data->cell3_voltage;
  dev->data->cell4_voltage            = new_data->cell4_voltage;
  dev->data->cell5_voltage            = new_data->cell5_voltage;
  dev->data->cell6_voltage            = new_data->cell6_voltage;

  leave_critical_section(flags);

  return buflen;
}

/****************************************************************************
 * Name: sbs_callback
 *
 * Description:
 *   Callback function that is to be invoked by the I2C slave driver when
 *   data has been received.  The received data will be checked against a
 *   list of registers that can be requested from a smart battery.  If there
 *   is a match the requested data will be put into the write buffer that is
 *   used by the I2C slave driver when a bus master wants to read the data.
 *
 * Input Parameters:
 *   arg - Pointer to the SBS device struct
 *
 * Returned Value:
 *   OK if a new write buffer was succesfully registered in response to the
 *   received command; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int sbs_callback(FAR void *arg)
{
  FAR struct sbs_dev_s *sbs_dev;
  int buffer_length;
  int i;

  /* Retrieve the pointer to the SBS device struct */

  sbs_dev = (FAR struct sbs_dev_s *)arg;
  DEBUGASSERT(sbs_dev && sbs_dev->i2c_slave_dev);

  /* Check which register was requested and prepare the write buffer */

  switch (sbs_dev->read_buffer[0])
    {
      case SBS_TEMPERATURE:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->temperature);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->temperature);
          buffer_length = 2;
        }
        break;

      case SBS_VOLTAGE:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->voltage);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->voltage);
          buffer_length = 2;
        }
        break;

      case SBS_CURRENT:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->current);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->current);
          buffer_length = 2;
        }
        break;

      case SBS_AVERAGE_CURRENT:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->average_current);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->average_current);
          buffer_length = 2;
        }
        break;

      case SBS_MAX_ERROR:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->max_error);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->max_error);
          buffer_length = 2;
        }
        break;

      case SBS_RELATIVE_STATE_OF_CHARGE:
        {
          sbs_dev->write_buffer[0] =
            LOBYTE(sbs_dev->data->relative_state_of_charge);
          sbs_dev->write_buffer[1] =
            HIBYTE(sbs_dev->data->relative_state_of_charge);
          buffer_length = 2;
        }
        break;

      case SBS_ABSOLUTE_STATE_OF_CHARGE:
        {
          sbs_dev->write_buffer[0] =
            LOBYTE(sbs_dev->data->absolute_state_of_charge);
          sbs_dev->write_buffer[1] =
            HIBYTE(sbs_dev->data->absolute_state_of_charge);
          buffer_length = 2;
        }
        break;

      case SBS_REMAINING_CAPACITY:
        {
          sbs_dev->write_buffer[0] =
            LOBYTE(sbs_dev->data->remaining_capacity);
          sbs_dev->write_buffer[1] =
            HIBYTE(sbs_dev->data->remaining_capacity);
          buffer_length = 2;
        }
        break;

      case SBS_FULL_CHARGE_CAPACITY:
        {
          sbs_dev->write_buffer[0] =
            LOBYTE(sbs_dev->data->full_charge_capacity);
          sbs_dev->write_buffer[1] =
            HIBYTE(sbs_dev->data->full_charge_capacity);
          buffer_length = 2;
        }
        break;

      case SBS_RUN_TIME_TO_EMPTY:
        {
          sbs_dev->write_buffer[0] =
            LOBYTE(sbs_dev->data->run_time_to_empty);
          sbs_dev->write_buffer[1] =
            HIBYTE(sbs_dev->data->run_time_to_empty);
          buffer_length = 2;
        }
        break;

      case SBS_AVERAGE_TIME_TO_EMPTY:
        {
          sbs_dev->write_buffer[0] =
            LOBYTE(sbs_dev->data->average_time_to_empty);
          sbs_dev->write_buffer[1] =
            HIBYTE(sbs_dev->data->average_time_to_empty);
          buffer_length = 2;
        }
        break;

      case SBS_CYCLE_COUNT:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->cycle_count);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->cycle_count);
          buffer_length = 2;
        }
        break;

      case SBS_DESIGN_CAPACITY:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->design_capacity);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->design_capacity);
          buffer_length = 2;
        }
        break;

      case SBS_DESIGN_VOLTAGE:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->design_voltage);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->design_voltage);
          buffer_length = 2;
        }
        break;

      case SBS_MANUFACTURE_DATE:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->manufacture_date);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->manufacture_date);
          buffer_length = 2;
        }
        break;

      case SBS_SERIAL_NUMBER:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->serial_number);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->serial_number);
          buffer_length = 2;
        }
        break;

      case SBS_MANUFACTURER_NAME:
        {
          DEBUGASSERT(sbs_dev->data->manufacturer_name);

          /* Determine how many characters to put into the write buffer, but
           * it can be at most 15.  The buffer can hold 16 bytes, including
           * the first byte that indicates the length of the string.
           */

          sbs_dev->write_buffer[0] =
            strnlen(sbs_dev->data->manufacturer_name, 15);

          /* Fill the write buffer */

          for (i = 0; i < sbs_dev->write_buffer[0]; i++)
            {
              sbs_dev->write_buffer[i + 1] =
                (uint8_t) sbs_dev->data->manufacturer_name[i];
            }

          buffer_length = (sbs_dev->write_buffer[0] + 1);
        }
        break;

      case SBS_DEVICE_NAME:
        {
          DEBUGASSERT(sbs_dev->data->device_name);

          /* Determine how many characters to put into the write buffer, but
           * it can be at most 15.  The buffer can hold 16 bytes, including
           * the first byte that indicates the length of the string.
           */

          sbs_dev->write_buffer[0] = strnlen(sbs_dev->data->device_name, 15);

          /* Fill the write buffer */

          for (i = 0; i < sbs_dev->write_buffer[0]; i++)
            {
              sbs_dev->write_buffer[i + 1] =
                (uint8_t) sbs_dev->data->device_name[i];
            }

          buffer_length = (sbs_dev->write_buffer[0] + 1);
        }
        break;

      case SBS_DEVICE_CHEMISTRY:
        {
          DEBUGASSERT(sbs_dev->data->device_chemistry);

          /* Determine how many characters to put into the write buffer, but
           * it can be at most 15.  The buffer can hold 16 bytes, including
           * the first byte that indicates the length of the string.
           */

          sbs_dev->write_buffer[0] =
            strnlen(sbs_dev->data->device_chemistry, 15);

          /* Fill the write buffer */

          for (i = 0; i < sbs_dev->write_buffer[0]; i++)
            {
              sbs_dev->write_buffer[i + 1] =
                (uint8_t) sbs_dev->data->device_chemistry[i];
            }

          buffer_length = (sbs_dev->write_buffer[0] + 1);
        }
        break;

      case SBS_MANUFACTURER_DATA:
        {
          DEBUGASSERT(sbs_dev->data->manufacturer_data);

          /* Determine how many bytes to put into the write buffer */

          sbs_dev->write_buffer[0] = sbs_dev->data->manufacturer_data_length;
          if (sbs_dev->write_buffer[0] > 15)
            {
              sbs_dev->write_buffer[0] = 15;

              /* The write buffer can only hold 16 bytes, including the first
               * byte that indicates the length of the byte array.
               * The array has to be limited to 15 bytes if it is longer.
               */
            }

          /* Fill the write buffer */

          for (i = 0; i < sbs_dev->write_buffer[0]; i++)
            {
              sbs_dev->write_buffer[i + 1] =
                sbs_dev->data->manufacturer_data[i];
            }

          buffer_length = (sbs_dev->write_buffer[0] + 1);
        }
        break;

      case SBS_CELL6_VOLTAGE:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->cell6_voltage);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->cell6_voltage);
          buffer_length = 2;
        }
        break;

      case SBS_CELL5_VOLTAGE:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->cell5_voltage);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->cell5_voltage);
          buffer_length = 2;
        }
        break;

      case SBS_CELL4_VOLTAGE:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->cell4_voltage);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->cell4_voltage);
          buffer_length = 2;
        }
        break;

      case SBS_CELL3_VOLTAGE:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->cell3_voltage);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->cell3_voltage);
          buffer_length = 2;
        }
        break;

      case SBS_CELL2_VOLTAGE:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->cell2_voltage);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->cell2_voltage);
          buffer_length = 2;
        }
        break;

      case SBS_CELL1_VOLTAGE:
        {
          sbs_dev->write_buffer[0] = LOBYTE(sbs_dev->data->cell1_voltage);
          sbs_dev->write_buffer[1] = HIBYTE(sbs_dev->data->cell1_voltage);
          buffer_length = 2;
        }
        break;

      default:
        {
          /* Set an empty write buffer with zero length if the requested
           * register does not exist (or is not yet supported).  The I2C
           * slave will usually handle an empty buffer by sending zero bytes.
           */

          return I2CS_WRITE(sbs_dev->i2c_slave_dev, NULL, 0);
        }
        break;
    }

  /* Install the (re)filled write buffer.  Technically this buffer needs to
   * be constant, but we want to be able to re-use the same buffer for the
   * next request, so we just cast the buffer to const.  This should not
   * cause any problems, because the write buffer is only changed when the
   * I2C slave driver invokes this callback, which only happens when a new
   * request has been received.
   */

  return I2CS_WRITE(sbs_dev->i2c_slave_dev,
                    (const uint8_t *)sbs_dev->write_buffer, buffer_length);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sbs_initialize
 *
 * Description:
 *   Create and register a Smart Battery System character driver.
 *
 *   This "Simple SBS" character driver supports (a subset of) the Smart
 *   Battery Data Specification, Revision 1.1.  This driver provides a buffer
 *   to the I2C slave driver.  This buffer can be updated at regular
 *   intervals by a user-space application.
 *
 * Input Parameters:
 *   minor         - The SBS character device will be registered as
 *                   /dev/sbsN where N is the minor number
 *   i2c_slave_dev - An instance of the lower half I2C slave driver
 *
 * Returned Value:
 *   OK if the driver was successfully registered; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int sbs_initialize(int minor, FAR struct i2c_slave_s *i2c_slave_dev)
{
  irqstate_t flags;
  struct sbs_dev_s *sbs_dev;
  char dev_name[16];
  int ret;

  /* Make sure the initialization is not interrupted */

  flags = enter_critical_section();

  /* Allocate an SBS device structure */

  sbs_dev = (struct sbs_dev_s *)kmm_zalloc(sizeof(struct sbs_dev_s));
  if (sbs_dev == NULL)
    {
      leave_critical_section(flags);
      return -ENOMEM;
    }
  else
    {
      /* Create the device name string */

      snprintf(dev_name, 16, "/dev/sbs%d", minor);

      /* Register the driver.  The associated private data is a reference to
       * the SBS device structure.
       */

      ret = register_driver(dev_name, &sbs_fops, 0, sbs_dev);
      if (ret < 0)
        {
          ferr("register_driver failed: %d\n", -ret);
          kmm_free(sbs_dev);

          leave_critical_section(flags);
          return ret;
        }
    }

  /* Allocate the SBS data structure */

  sbs_dev->data = (struct sbs_data_s *)kmm_zalloc(sizeof(struct sbs_data_s));
  if (sbs_dev->data == NULL)
    {
      leave_critical_section(flags);
      return -ENOMEM;
    }

  /* Set-up the I2C slave device.  Install a read-buffer as well as a
   * callback, which will receive the SBS device structure as an argument.
   */

  DEBUGASSERT(i2c_slave_dev);
  sbs_dev->i2c_slave_dev = i2c_slave_dev;

  ret = I2CS_READ(sbs_dev->i2c_slave_dev, sbs_dev->read_buffer, 3);
  if (ret < 0)
    {
      leave_critical_section(flags);
      return ret;
    }

  ret = I2CS_REGISTERCALLBACK(sbs_dev->i2c_slave_dev, sbs_callback,
                              (void *)sbs_dev);
  if (ret < 0)
    {
      leave_critical_section(flags);
      return ret;
    }

  leave_critical_section(flags);
  return 0;
}

#endif /* CONFIG_SBS_DRIVER */
