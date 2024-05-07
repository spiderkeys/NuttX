/****************************************************************************
 * boards/arm/s32k1xx/rddrone-bms772/src/s32k1xx_gpio.c
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017 Alan Carvalho de Assis. All rights reserved.
 *   Author:  Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Based on: boards/imxrt1050-evk/src/imxrt_gpio.c
 *
 *   Author:  Pavlina Koleva <pavlinaikoleva19@gmail.com>
 *   Modified by: Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
 *
 * Also based on: boards/arm/stm32/stm32f103-minimum/src/stm32_gpio.c
 *
 *   Author:  Alan Carvalho de Assis <acassis@gmail.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "chip.h"

#include <s32k1xx_pin.h>
#include "rddrone-bms772.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct s32k1xx_gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct s32k1xx_gpint_dev_s
{
  struct s32k1xx_gpio_dev_s s32k1xx_gpio;
  pin_interrupt_t callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
#if NUM_OF_GPIO_IN > 0 
static int gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int gpin_setpintype(FAR struct gpio_dev_s *dev,
                            enum gpio_pintype_e pintype);
#endif

#if NUM_OF_GPIO_OUT > 0 
static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int gpout_write(FAR struct gpio_dev_s *dev, bool value);
#endif

#if NUM_OF_GPIO_INT > 0 
static int gpint_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int gpint_attach(FAR struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
static int gpint_enable(FAR struct gpio_dev_s *dev, bool enable);
static int gpint_setpintype(FAR struct gpio_dev_s *dev,
                            enum gpio_pintype_e pintype);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
#if NUM_OF_GPIO_IN > 0 
static const struct gpio_operations_s gpin_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
  .go_setpintype = gpin_setpintype,
};
#endif

#if NUM_OF_GPIO_OUT > 0 
static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
  .go_setpintype = NULL,
};
#endif

#if NUM_OF_GPIO_INT > 0 
static const struct gpio_operations_s gpint_ops =
{
  .go_read   = gpint_read,
  .go_write  = NULL,
  .go_attach = gpint_attach,
  .go_enable = gpint_enable,
  .go_setpintype = gpint_setpintype,
};
#endif

#if NUM_OF_GPIO_IN > 0 
static struct s32k1xx_gpio_dev_s g_gpin[NUM_OF_GPIO_IN];

/* WARNING IF THIS CHANGES CHANGE gpio.h 
 *      and the NUM_OF_GPIO_x
 *      and the array in s32k1xx_gpio.c 
 */

/* This array maps the GPIO pins used as INPUT */

static const uint32_t g_gpioinputs[NUM_OF_GPIO_IN] =
{
  /* GPIO_IN0, GPIO_IN1, GPIO_IN2, GPIO_IN3, GPIO_IN4 */
};
#endif

#if NUM_OF_GPIO_OUT > 0 
static struct s32k1xx_gpio_dev_s g_gpout[NUM_OF_GPIO_OUT];

/* WARNING IF THIS CHANGES CHANGE gpio.h 
 *      and the NUM_OF_GPIO_x
 *      and the array in s32k1xx_gpio.c 
 */

/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[NUM_OF_GPIO_OUT] =
{
  GPIO_OUT0, GPIO_OUT1, GPIO_OUT2, GPIO_OUT3, GPIO_OUT4 /* , GPIO_OUT5, GPIO_OUT6, GPIO_OUT7, GPIO_OUT8, GPIO_OUT9 */
};
#endif

#if NUM_OF_GPIO_INT > 0 
static struct s32k1xx_gpint_dev_s g_gpint[NUM_OF_GPIO_INT];

/* WARNING IF THIS CHANGES CHANGE gpio.h 
 *      and the NUM_OF_GPIO_x
 *      and the array in s32k1xx_gpio.c 
 */

/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiointinputs[NUM_OF_GPIO_INT] =
{
  GPIO_INT5, GPIO_INT6, GPIO_INT7, GPIO_INT8, GPIO_INT9, GPIO_INT10, GPIO_INT11 /* ,GPIO_INT12, GPIO_INT13, */
};
#endif

/* #if NUM_OF_GPIO_IN > 0
 * static const uint32_t g_gpioinputs[NUM_OF_GPIO_IN] =
 * {
 *   GPIO_IN1,
 * };
 * static struct s32k1xx_gpio_dev_s g_gpin[NUM_OF_GPIO_IN];
 * #endif
 */ 

 /* This array maps the GPIO pins used as OUTPUT */

/* #if NUM_OF_GPIO_OUT > 0
 * static const uint32_t g_gpiooutputs[NUM_OF_GPIO_OUT] =
 * {
 *   GPIO_OUT1, GPIO_OUT2
 * };
 * static struct s32k1xx_gpio_dev_s g_gpout[NUM_OF_GPIO_OUT];
 * #endif
 */ 

 /* This array maps the GPIO pins used as interrupt pins */

/* #if NUM_OF_GPIO_INT > 0
 * static const uint32_t g_gpiointinputs[NUM_OF_GPIO_INT] =
 * {
 *   GPIO_INT1,
 * };
 * static struct s32k1xx_gpint_dev_s g_gpint[NUM_OF_GPIO_INT];
 * #endif
 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/
#if NUM_OF_GPIO_INT > 0 
static int s32k1xx_gpio_interrupt(int irq, void *context, void *arg)
{
  FAR struct s32k1xx_gpint_dev_s *s32k1xx_gpint =
    (FAR struct s32k1xx_gpint_dev_s *)arg;

  DEBUGASSERT(s32k1xx_gpint != NULL && s32k1xx_gpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", s32k1xx_gpint->callback);

  s32k1xx_gpint->callback(&s32k1xx_gpint->s32k1xx_gpio.gpio,
                           s32k1xx_gpint->s32k1xx_gpio.id);
  return OK;
}
#endif

#if NUM_OF_GPIO_IN > 0 
static int gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct s32k1xx_gpio_dev_s *s32k1xx_gpio =
    (FAR struct s32k1xx_gpio_dev_s *)dev;

  DEBUGASSERT(s32k1xx_gpio != NULL && value != NULL);
  DEBUGASSERT(s32k1xx_gpio->id < NUM_OF_GPIO_IN);
  gpioinfo("Reading...\n");

  *value = s32k1xx_gpioread(g_gpioinputs[s32k1xx_gpio->id]);
  return OK;
}

static int gpin_setpintype(FAR struct gpio_dev_s *dev,
                            enum gpio_pintype_e pintype)
{
  int ret = !OK;
  unsigned int pinconfig;

  FAR struct s32k1xx_gpio_dev_s *s32k1xx_gpio =
    (FAR struct s32k1xx_gpio_dev_s *)dev;

  DEBUGASSERT(s32k1xx_gpio != NULL);
  DEBUGASSERT(s32k1xx_gpio->id < NUM_OF_GPIO_IN);
  gpioinfo("Setpintype...\n");

  /* Clear the pinmode */

  pinconfig = (g_gpioinputs[s32k1xx_gpio->id]) & (~(_PIN_MODE_MASK));

  /* Add the correct new pintype the pintype */

  switch(pintype)
  {
    case GPIO_INPUT_PIN:
      /* Add the new pinmode */

      pinconfig |= GPIO_INPUT;
    break;
    case GPIO_INPUT_PIN_PULLUP:
      /* Add the new pinmode */

      pinconfig |= GPIO_PULLUP;
    break;
    case GPIO_INPUT_PIN_PULLDOWN:
      /* Add the new pinmode */

      pinconfig |= GPIO_PULLDOWN;
    break;
    case GPIO_INTERRUPT_BOTH_PIN:
      /* Add the new pinmode */

      pinconfig |= (GPIO_INPUT | PIN_INT_BOTH);
    break;
    default: 
      /* Not implemented yet */

      DEBUGASSERT(NULL);

      /* Set the pinconfig to 0 to trigger on */

      pinconfig = 0;

    break;
  }

  /* Check if the pinconfig is set */

  if(pinconfig != 0)
  {
    /* Change the pintype */

    ret = s32k1xx_pinconfig(pinconfig);
  }

  /* Return */

  return ret;
}
#endif

#if NUM_OF_GPIO_OUT > 0 
static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct s32k1xx_gpio_dev_s *s32k1xx_gpio =
    (FAR struct s32k1xx_gpio_dev_s *)dev;

  DEBUGASSERT(s32k1xx_gpio != NULL && value != NULL);
  DEBUGASSERT(s32k1xx_gpio->id < NUM_OF_GPIO_OUT);
  gpioinfo("Reading...\n");

  *value = s32k1xx_gpioread(g_gpiooutputs[s32k1xx_gpio->id]);
  return OK;
}

static int gpout_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct s32k1xx_gpio_dev_s *s32k1xx_gpio =
    (FAR struct s32k1xx_gpio_dev_s *)dev;

  DEBUGASSERT(s32k1xx_gpio != NULL);
  DEBUGASSERT(s32k1xx_gpio->id < NUM_OF_GPIO_OUT);
  gpioinfo("Writing %d\n", (int)value);

  s32k1xx_gpiowrite(g_gpiooutputs[s32k1xx_gpio->id], value);
  return OK;
}

#endif

#if NUM_OF_GPIO_INT > 0 
static int gpint_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct s32k1xx_gpint_dev_s *s32k1xx_gpint =
    (FAR struct s32k1xx_gpint_dev_s *)dev;

  DEBUGASSERT(s32k1xx_gpint != NULL && value != NULL);
  DEBUGASSERT(s32k1xx_gpint->s32k1xx_gpio.id < NUM_OF_GPIO_INT);
  gpioinfo("Reading int pin...\n");

  *value = s32k1xx_gpioread(g_gpiointinputs[s32k1xx_gpint->s32k1xx_gpio.id]);
  return OK;
}

static int gpint_attach(FAR struct gpio_dev_s *dev, pin_interrupt_t callback)
{
  FAR struct s32k1xx_gpint_dev_s *s32k1xx_gpint =
    (FAR struct s32k1xx_gpint_dev_s *)dev;

  gpioinfo("Attaching the callback\n");
  s32k1xx_pinirqattach(g_gpiointinputs[s32k1xx_gpint->s32k1xx_gpio.id],
                       s32k1xx_gpio_interrupt,
                       &g_gpint[s32k1xx_gpint->s32k1xx_gpio.id]);

  gpioinfo("Attach %p\n", callback);
  s32k1xx_gpint->callback = callback;
  return OK;
}

static int gpint_enable(FAR struct gpio_dev_s *dev, bool enable)
{
  FAR struct s32k1xx_gpint_dev_s *s32k1xx_gpint =
    (FAR struct s32k1xx_gpint_dev_s *)dev;

  if (enable)
    {
      if (s32k1xx_gpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");
          s32k1xx_pinirqenable(
            g_gpiointinputs[s32k1xx_gpint->s32k1xx_gpio.id]);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      s32k1xx_pinirqdisable(g_gpiointinputs[s32k1xx_gpint->s32k1xx_gpio.id]);
    }

  return OK;
}

static int gpint_setpintype(FAR struct gpio_dev_s *dev,
                            enum gpio_pintype_e pintype)
{
  int ret = !OK;
  unsigned int pinconfig;

  FAR struct s32k1xx_gpint_dev_s *s32k1xx_gpint =
    (FAR struct s32k1xx_gpint_dev_s *)dev;

  DEBUGASSERT(s32k1xx_gpint != NULL);
  DEBUGASSERT(s32k1xx_gpint->s32k1xx_gpio.id < NUM_OF_GPIO_INT);
  gpioinfo("Setpintype intpin...\n");

  /* Clear the pinmode */

  pinconfig = (g_gpiointinputs[s32k1xx_gpint->s32k1xx_gpio.id]) & (~(_PIN_MODE_MASK));

  /* Add the correct new pintype the pintype */

  switch(pintype)
  {
    case GPIO_INPUT_PIN:
      /* Add the new pinmode */

      pinconfig |= GPIO_INPUT;
    break;
    case GPIO_INPUT_PIN_PULLUP:
      /* Add the new pinmode */

      pinconfig |= GPIO_PULLUP;
    break;
    case GPIO_INPUT_PIN_PULLDOWN:
      /* Add the new pinmode */

      pinconfig |= GPIO_PULLDOWN;
    break;
    case GPIO_INTERRUPT_BOTH_PIN:
      /* Add the new pinmode */

      pinconfig |= (GPIO_INPUT | PIN_INT_BOTH);
    break;
    default: 
      /* Not implemented yet */

      DEBUGASSERT(NULL);

      /* Set the pinconfig to 0 to trigger on */

      pinconfig = 0;

    break;
  }

  /* Check if the pinconfig is set */

  if(pinconfig != 0)
  {
    /* Change the pintype */

    ret = s32k1xx_pinconfig(pinconfig);
  }

  /* Return */

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int s32k1xx_gpio_initialize(void)
{
  int i;
  int pincount = 0;

#if NUM_OF_GPIO_IN > 0
  for (i = 0; i < NUM_OF_GPIO_IN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;

      gpio_pin_register(&g_gpin[i].gpio, pincount);

      /* Configure the pin that will be used as input */

      s32k1xx_pinconfig(g_gpioinputs[i]);

      pincount++;
    }
#endif

#if NUM_OF_GPIO_OUT > 0
  for (i = 0; i < NUM_OF_GPIO_OUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;

      gpio_pin_register(&g_gpout[i].gpio, pincount);

      /* Configure the pin that will be used as output */

      s32k1xx_gpiowrite(g_gpiooutputs[i], 0);
      s32k1xx_pinconfig(g_gpiooutputs[i]);

      pincount++;
    }
#endif

#if NUM_OF_GPIO_INT > 0
  for (i = 0; i < NUM_OF_GPIO_INT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].s32k1xx_gpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].s32k1xx_gpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].s32k1xx_gpio.id              = i;

      gpio_pin_register(&g_gpint[i].s32k1xx_gpio.gpio, pincount);

      /* Configure the pin that will be used as interrupt input */

      s32k1xx_pinconfig(g_gpiointinputs[i]);

      pincount++;
    }
#endif

  return 0;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
