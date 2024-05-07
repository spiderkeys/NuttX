/****************************************************************************
 * boards/arm/s32k1xx/rddrone-bms772/src/rddrone-bms772.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_S32K1XX_RDDRONE_BMS772_SRC_RDDRONE_BMS772_H
#define __BOARDS_ARM_S32K1XX_RDDRONE_BMS772_SRC_RDDRONE_BMS772_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_pin.h"

#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* RDDRONE-BMS772 GPIOs *****************************************************/

/* LEDs.  The RDDRONE-BMS772 has one RGB LED:
 *
 *   RedLED   PTD16 (FTM0 CH1)
 *   GreenLED PTB13 (FTM0 CH1)
 *   BlueLED  PTD15 (FTM0 CH0)
 *
 * An output of '1' illuminates the LED.
 */

#define GPIO_LED_R     (PIN_PTD16 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED_G     (PIN_PTB13 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED_B     (PIN_PTD15 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)

/* Buttons.  The rddrone-bms772 supports one button:
 *
 *   SW1  PTC14
 */

//#define GPIO_SW1       (PIN_PTC14 | PIN_INT_BOTH)    
//#define GPIO_SW3       (PIN_PTC13 | PIN_INT_BOTH)

/* GPIO pins to be registered to the GPIO driver: */

/* WARNING IF THIS CHANGES CHANGE gpio.h 
 *      and the NUM_OF_GPIO_x
 *      and the array in s32k1xx_gpio.c 
 */

#define NUM_OF_GPIO_IN  0 /* Amount of GPIO input pins */
#define NUM_OF_GPIO_OUT 5 /* Amount of GPIO output pins */
#ifdef CONFIG_S32K1XX_GPIOIRQ
#define NUM_OF_GPIO_INT 7 /* Amount of GPIO interrupt pins */
#else
#define NUM_OF_GPIO_INT 0 /* Amount of GPIO interrupt pins */
#endif

/* WARNING IF THIS CHANGES CHANGE gpio.h 
 *      and the NUM_OF_GPIO_x
 *      and the array in s32k1xx_gpio.c 
 */

/* #define GPIO_IN0   (PIN_PTA13 | GPIO_INPUT) // NFC_ED */
/* #define GPIO_IN1   (PIN_PTC14 | GPIO_INPUT) // SBC Wake */
/* #define GPIO_IN2   (PIN_PTC8  | GPIO_INPUT) // GATE_RS */
/* #define GPIO_IN3   (PIN_PTA11 | GPIO_INPUT) // SBC_LIMP */
/* #define GPIO_IN0   (PIN_PTC9  | GPIO_INPUT) // BCC_FAULT */

/* #define GPIO_OUT1  (PIN_PTC1  | GPIO_OUTPUT) // GATE_CTRL_CP */
/* #define GPIO_OUT2  (PIN_PTC2  | GPIO_OUTPUT) // GATE_CTRL_D */
/* #define GPIO_OUT3  (PIN_PTD5  | GPIO_OUTPUT) // BCC_RESET */
/* #define GPIO_OUT4  (PIN_PTA12 | GPIO_OUTPUT) // NFC_HPD */
/* #define GPIO_OUT5  (PIN_PTC15 | GPIO_OUTPUT) // AUTH_WAKE */

#define GPIO_OUT0   (PIN_PTC1  | GPIO_OUTPUT) /* GATE_CTRL_CP */
#define GPIO_OUT1   (PIN_PTC2  | GPIO_OUTPUT) /* GATE_CTRL_D */
#define GPIO_OUT2   (PIN_PTD5  | GPIO_OUTPUT) /* BCC_RESET */
#define GPIO_OUT3   (PIN_PTA12 | GPIO_OUTPUT) /* NFC_HPD */
#define GPIO_OUT4   (PIN_PTC15 | GPIO_OUTPUT) /* AUTH_WAKE */

#define GPIO_INT5   (PIN_PTE8  | GPIO_INPUT | PIN_INT_BOTH) /* EXT_OUT1 To ext header */
#define GPIO_INT6   (PIN_PTC3  | GPIO_INPUT | PIN_INT_BOTH) /* OVERCURRENT */
#define GPIO_INT7   (PIN_PTC14 | GPIO_INPUT | PIN_INT_BOTH) /* SBC Wake  */
#define GPIO_INT8   (PIN_PTC8  | GPIO_INPUT | PIN_INT_BOTH) /* GATE_RS */
#define GPIO_INT9   (PIN_PTA11 | GPIO_INPUT | PIN_INT_BOTH) /* SBC_LIMP */
#define GPIO_INT10  (PIN_PTC9  | GPIO_INPUT | PIN_INT_BOTH) /* BCC_FAULT */
#define GPIO_INT11  (PIN_PTA13 | GPIO_INPUT | PIN_INT_BOTH) /* NFC_ED */

/* SPI chip selects */

/* Count of peripheral clock user configurations */

#define NUM_OF_PERIPHERAL_CLOCKS_0 12

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* User peripheral configuration structure 0 */

extern const struct peripheral_clock_config_s g_peripheral_clockconfig0[];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int s32k1xx_bringup(void);

#ifdef CONFIG_DEV_GPIO
/****************************************************************************
 * Name: s32k1xx_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int s32k1xx_gpio_initialize(void);
#endif

#ifdef CONFIG_S32K1XX_LPI2C
/****************************************************************************
 * Name: s32k1xx_smartbattery_initialize
 *
 * Description:
 *   Initialize smart battery SMBus (I2C slave) example.
 *
 ****************************************************************************/

int s32k1xx_smartbattery_initialize(void);
#endif

/****************************************************************************
 * Name: s32k1xx_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the RDDRONE-BMS772
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_LPSPI
void s32k1xx_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: s32k1xx_nrstcheck
 *
 * Description:
 *   It will make the NRST pin a pull-down GPIO input pin and check the state
 *   This will be saved in a block device and the pin is configured as NRST.  
 *
 ****************************************************************************/
#if defined(CONFIG_NRST_CHECK_PROC_FS) && defined(CONFIG_FS_PROCFS)
int s32k1xx_nrstcheck(void);
#endif

/****************************************************************************
 * Name: board_lcd_initialize_I2C
 *
 * Description:
 *   Initialize SSD1306 drivers for use with LCD
 *
 ****************************************************************************/
#ifdef CONFIG_LCD
int board_lcd_initialize_I2C(FAR struct i2c_master_s * i2cInit);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_S32K1XX_RDDRONE_BMS772_SRC_RDDRONE_BMS772_H */
