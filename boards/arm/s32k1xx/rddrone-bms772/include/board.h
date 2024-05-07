/****************************************************************************
 * boards/arm/s32k1xx/rddrone-bms772/include/board.h
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

#ifndef __BOARDS_ARM_RDDRONE_BMS772_INCLUDE_BOARD_H
#define __BOARDS_ARM_RDDRONE_BMS772_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The RDDRONE-BMS772 is fitted with a 32 MHz Crystal */

#define BOARD_XTAL_FREQUENCY 32000000

/* The S32K144 will run at 80 MHz */

/* LED definitions **********************************************************/

/* The RDDRONE-BMS772 has one RGB LED:
 *
 *   RedLED   PTD16 (FTM0 CH1)
 *   GreenLED PTB13 (FTM0 CH1)
 *   BlueLED  PTD15 (FTM0 CH0)
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual RGB
 * components.
 *
 * The RGB components could, alternatively be controlled through PWM using
 * the common RGB LED driver.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_R       0
#define BOARD_LED_G       1
#define BOARD_LED_B       2
#define BOARD_NLEDS       3

/* LED bits for use with board_userled_all() */

#define BOARD_LED_R_BIT   (1 << BOARD_LED_R)
#define BOARD_LED_G_BIT   (1 << BOARD_LED_G)
#define BOARD_LED_B_BIT   (1 << BOARD_LED_B)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board
 * the RDDRONE-BMS772.  The following definitions describe how NuttX
 * controls the LEDs:
 *
 *   SYMBOL                Meaning                      LED state
 *                                                      RED   GREEN  BLUE
 *   -------------------  ----------------------------  -----------------
 */

#define LED_STARTED       1 /* NuttX has been started    OFF   OFF    OFF */
#define LED_HEAPALLOCATE  2 /* Heap has been allocated   OFF   OFF    ON  */
#define LED_IRQSENABLED   0 /* Interrupts enabled        OFF   OFF    ON  */
#define LED_STACKCREATED  3 /* Idle stack created        OFF   ON     OFF */
#define LED_INIRQ         0 /* In an interrupt          (no change)       */
#define LED_SIGNAL        0 /* In a signal handler      (no change)       */
#define LED_ASSERTION     0 /* An assertion failed      (no change)       */
#define LED_PANIC         4 /* The system has crashed    FLASH OFF    OFF */
#undef  LED_IDLE            /* RDDRONE-BMS772 in sleep mode (Not used)    */

/* Button definitions *******************************************************/

/* The rddrone-bms772 supports one button:
 *
 *   SW1  PTC14
 */

#define BUTTON_SW1         0

#define NUM_BUTTONS        1

#define BUTTON_SW1_BIT    (1 << BUTTON_SW1)

/* Alternate function pin selections ****************************************/

/* By default, the serial console will be provided on the OpenSDA VCOM port:
 *
 *   OpenSDA UART TX  PTC7 (LPUART1_TX)
 *   OpenSDA UART RX  PTC6 (LPUART1_RX)
 */

/* optional UART on connector J21                   */
#define PIN_LPUART0_RX    PIN_LPUART0_RX_1  /* PTB0 */
#define PIN_LPUART0_TX    PIN_LPUART0_TX_1  /* PTB1 */

/* NuttX console                                    */
#define PIN_LPUART1_RX    PIN_LPUART1_RX_1  /* PTC6 */
#define PIN_LPUART1_TX    PIN_LPUART1_TX_1  /* PTC7 */

/* SPI selections ***********************************************************/

/* SBC connection through SPI 0                     */
#define PIN_LPSPI0_SCK   PIN_LPSPI0_SCK_2   /* PTB2 */
#define PIN_LPSPI0_MISO  PIN_LPSPI0_SIN_2   /* PTB3 */
#define PIN_LPSPI0_MOSI  PIN_LPSPI0_SOUT_3  /* PTB4 */
#define PIN_LPSPI0_PCS   PIN_LPSPI0_PCS0_2 // don't make it a GPIO 

/* BCC connection through SPI 1                     */
#define PIN_LPSPI1_SCK   PIN_LPSPI1_SCK_2   /* PTD0 */
#define PIN_LPSPI1_MISO  PIN_LPSPI1_SIN_2   /* PTD1 */
#define PIN_LPSPI1_MOSI  PIN_LPSPI1_SOUT_2  /* PTD2 */
#define PIN_LPSPI1_PCS   PIN_LPSPI1_PCS0_1 // don't make it a GPIO 

/* I2C selections ***********************************************************/

#define PIN_LPI2C0_SCL   PIN_LPI2C0_SCL_2   /* PTA3 */
#define PIN_LPI2C0_SDA	 PIN_LPI2C0_SDA_2   /* PTA2 */

/* I2C slave selections *****************************************************/

#define PIN_LPI2C0S_SCL  PIN_LPI2C0_SCLS_1   /* PTA0 */
#define PIN_LPI2C0S_SDA	 PIN_LPI2C0_SDAS_1   /* PTA1 */

/* CAN selections ***********************************************************/
#define PIN_CAN0_TX      PIN_CAN0_TX_3      /* PTE5 */
#define PIN_CAN0_RX      PIN_CAN0_RX_3      /* PTE4 */

#endif  /* __BOARDS_ARM_RDDRONE_BMS772_INCLUDE_BOARD_H */
