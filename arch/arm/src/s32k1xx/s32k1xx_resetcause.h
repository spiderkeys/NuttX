/******************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_resetcause.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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
 ******************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_RESETCAUSE_H
#define __ARCH_ARM_SRC_S32K1XX_RESETCAUSE_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

// #include "arm_internal.h"
// #include "s32k1xx_config.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/*!
 * @brief 	this function initializes the resetcause
 *
 * 			It will get the resetcause and store it
 * 			
 * @param 	none
 *
 * @return 	none
 *
 */
void s32k1xx_resetcause_init(void);

/*!
 * @brief   this function registrates the reset cause as a proc fs
 *      
 * @param   none
 *
 * @return  0 if OK, error number otherwise
 *
 */
int s32k1xx_resetcause_initialize_procfs(void);

#endif /* __ARCH_ARM_SRC_S32K1XX_RESETCAUSE_H */
