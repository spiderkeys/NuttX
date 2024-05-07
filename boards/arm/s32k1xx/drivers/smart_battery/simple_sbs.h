/****************************************************************************
 * boards/arm/s32k1xx/drivers/smart_battery/simple_sbs.h
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

#ifndef __BOARDS_ARM_S32K1XX_DRIVERS_SMART_BATTERY_SIMPLE_SBS_H
#define __BOARDS_ARM_S32K1XX_DRIVERS_SMART_BATTERY_SIMPLE_SBS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#ifdef CONFIG_SBS_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Smart Battery Data Specification v1.1 registers **************************/

/* NOT YET IMPLEMENTED! */

#if 0
#define SBS_MANUFACTURER_ACCESS         0x00
#define SBS_REMAINING_CAPACITY_ALARM    0x01
#define SBS_REMAINING_TIME_ALARM        0x02
#define SBS_BATTERY_MODE                0x03
#define SBS_AT_RATE                     0x04
#define SBS_AT_RATE_TIME_TO_FULL        0x05
#define SBS_AT_RATE_TIME_TO_EMPTY       0x06
#define SBS_AT_RATE_OK                  0x07
#endif

#define SBS_TEMPERATURE                 0x08
#define SBS_VOLTAGE                     0x09
#define SBS_CURRENT                     0x0a
#define SBS_AVERAGE_CURRENT             0x0b
#define SBS_MAX_ERROR                   0x0c
#define SBS_RELATIVE_STATE_OF_CHARGE    0x0d
#define SBS_ABSOLUTE_STATE_OF_CHARGE    0x0e
#define SBS_REMAINING_CAPACITY          0x0f
#define SBS_FULL_CHARGE_CAPACITY        0x10
#define SBS_RUN_TIME_TO_EMPTY           0x11
#define SBS_AVERAGE_TIME_TO_EMPTY       0x12

/* NOT YET IMPLEMENTED! */

#if 0
#define SBS_AVERAGE_TIME_TO_FULL        0x13
#define SBS_CHARGING_CURRENT            0x14
#define SBS_CHARGING_VOLTAGE            0x15
#define SBS_BATTERY_STATUS              0x16
#endif

#define SBS_CYCLE_COUNT                 0x17
#define SBS_DESIGN_CAPACITY             0x18
#define SBS_DESIGN_VOLTAGE              0x19

/* NOT YET IMPLEMENTED! */

#if 0
#define SBS_SPECIFICATION_INFO          0x1a
#endif

#define SBS_MANUFACTURE_DATE            0x1b
#define SBS_SERIAL_NUMBER               0x1c
#define SBS_MANUFACTURER_NAME           0x20
#define SBS_DEVICE_NAME                 0x21
#define SBS_DEVICE_CHEMISTRY            0x22
#define SBS_MANUFACTURER_DATA           0x23

/* Non-standard registers ***************************************************/

#define SBS_CELL6_VOLTAGE               0x3a
#define SBS_CELL5_VOLTAGE               0x3b
#define SBS_CELL4_VOLTAGE               0x3c
#define SBS_CELL3_VOLTAGE               0x3d
#define SBS_CELL2_VOLTAGE               0x3e
#define SBS_CELL1_VOLTAGE               0x3f

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Battery data */

struct sbs_data_s
{
  uint16_t temperature;              /* 0.1  K,       0 <->  6,553.5  K */
  uint16_t voltage;                  /* 1.0 mV,       0 <-> 65,535.0 mV */
  uint16_t current;                  /* 1.0 mA, -32,767 <-> 32,767.0 mA */
  uint16_t average_current;          /* 1.0 mA, -32,767 <-> 32,767.0 mA */
  uint16_t max_error;                /* 1.0  %,       0 <->    100.0  % */
  uint16_t relative_state_of_charge; /* 1.0  %,       0 <->    100.0  % */
  uint16_t absolute_state_of_charge; /* 1.0  %,       0 <->    100.0  % */
  uint16_t remaining_capacity;       /* 1.0 mAh,      0 <-> 65,535.0 mAh */
  uint16_t full_charge_capacity;     /* 1.0 mAh,      0 <-> 65,535.0 mAh */
  uint16_t run_time_to_empty;        /* 1.0 min,      0 <-> 65,535.0 min */
  uint16_t average_time_to_empty;    /* 1.0 min,      0 <-> 65,535.0 min */

  uint16_t cycle_count;              /* 1 cycle, 0 <-> 65,535 cycles */
  uint16_t design_capacity;          /* 1 mAh,   0 <-> 65,535 mAh */
  uint16_t design_voltage;           /* 1 mV,    0 <-> 65,535 mV */
  uint16_t manufacture_date;         /* (year-1980)*512 + month*32 + day */
  uint16_t serial_number;
  const char *manufacturer_name;
  const char *device_name;
  const char *device_chemistry;
  const uint8_t *manufacturer_data;
  uint8_t manufacturer_data_length;

  uint16_t cell1_voltage;            /* 1.0 mV,       0 <-> 65,535.0 mV */
  uint16_t cell2_voltage;            /* 1.0 mV,       0 <-> 65,535.0 mV */
  uint16_t cell3_voltage;            /* 1.0 mV,       0 <-> 65,535.0 mV */
  uint16_t cell4_voltage;            /* 1.0 mV,       0 <-> 65,535.0 mV */
  uint16_t cell5_voltage;            /* 1.0 mV,       0 <-> 65,535.0 mV */
  uint16_t cell6_voltage;            /* 1.0 mV,       0 <-> 65,535.0 mV */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

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

int sbs_initialize(int minor, FAR struct i2c_slave_s *i2c_slave_dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_SBS_DRIVER */
#endif /* __BOARDS_ARM_S32K1XX_DRIVERS_SMART_BATTERY_SIMPLE_SBS */