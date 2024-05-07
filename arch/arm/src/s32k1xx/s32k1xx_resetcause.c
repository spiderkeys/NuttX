/******************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_resetcause.c
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

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/kmalloc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <debug.h>

#include "arm_arch.h"

#include "hardware/s32k1xx_rcm.h"

// #include "s32k1xx_config.h"
#include "s32k1xx_resetcause.h"

#include "arm_internal.h"

#include <arch/board/board.h> /* Include last:  has dependencies */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Determines the size of an intermediate buffer that must be large enough
 * to handle the longest line generated by this logic.
 */
#define RESETCAUSE_LINELEN 6

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one open "file" */

struct resetcause_file_s
{
  struct procfs_file_s  base;        /* Base open file structure */
  unsigned int linesize;             /* Number of valid characters in line[] */
  char line[RESETCAUSE_LINELEN];     /* Pre-allocated buffer for formatted lines */
  unsigned int resetcause;           /* Variable representing the MCU specific reset cause */
};

static unsigned int gResetCause = 0;

#if defined(CONFIG_RESET_CAUSE_PROC_FS) && defined(CONFIG_FS_PROCFS)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* File system methods */

static int     resetcause_open(FAR struct file *filep, FAR const char *relpath,
                 int oflags, mode_t mode);
static int     resetcause_close(FAR struct file *filep);
static ssize_t resetcause_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);

static int     resetcause_dup(FAR const struct file *oldp,
                 FAR struct file *newp);

static int     resetcause_stat(FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct procfs_operations resetcause_operations =
{
  resetcause_open,   /* open */
  resetcause_close,  /* close */
  resetcause_read,   /* read */
  NULL,              /* write */

  resetcause_dup,    /* dup */

  NULL,              /* opendir */
  NULL,              /* closedir */
  NULL,              /* readdir */
  NULL,              /* rewinddir */

  resetcause_stat    /* stat */
};

static const struct procfs_entry_s g_resetcause_procfs =
{
  "resetcause", &resetcause_operations
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: resetcause_open
 ****************************************************************************/

static int resetcause_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct resetcause_file_s *attr;

  finfo("Open '%s'\n", relpath);

  /* PROCFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   *
   * REVISIT:  Write-able proc files could be quite useful.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* "resetcause" is the only acceptable value for the relpath */

  if (strcmp(relpath, "resetcause") != 0)
    {
      ferr("ERROR: relpath is '%s'\n", relpath);
      return -ENOENT;
    }

  /* Allocate a container to hold the file attributes */

  attr = kmm_zalloc(sizeof(struct resetcause_file_s));
  if (!attr)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* Save the attributes as the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)attr;
  return OK;
}

/****************************************************************************
 * Name: resetcause_close
 ****************************************************************************/

static int resetcause_close(FAR struct file *filep)
{
  FAR struct resetcause_file_s *attr;

  /* Recover our private data from the struct file instance */

  attr = (FAR struct resetcause_file_s *)filep->f_priv;
  DEBUGASSERT(attr);

  /* Release the file attributes structure */

  kmm_free(attr);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: resetcause_read
 ****************************************************************************/

static ssize_t resetcause_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct resetcause_file_s *attr;
  size_t linesize;
  off_t offset;
  ssize_t ret;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  attr = (FAR struct resetcause_file_s *)filep->f_priv;
  DEBUGASSERT(attr);

  /* Get the resetcause value and store it  */
  attr->resetcause = gResetCause;

  /* Convert the resetcause to a string */
  linesize  = snprintf(attr->line, RESETCAUSE_LINELEN, "0x%x", attr->resetcause);

  /* Save the linesize in case we are re-entered with f_pos > 0 */

  attr->linesize = linesize;

  /* Transfer the system reset cause to user receive buffer */

  offset = filep->f_pos;

  ret = procfs_memcpy(attr->line, attr->linesize, buffer, buflen, &offset);

  return ret;
}

/****************************************************************************
 * Name: resetcause_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int resetcause_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct resetcause_file_s *oldattr;
  FAR struct resetcause_file_s *newattr;

  finfo("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldattr = (FAR struct resetcause_file_s *)oldp->f_priv;
  DEBUGASSERT(oldattr);

  /* Allocate a new container to hold the task and attribute selection */

  newattr = kmm_malloc(sizeof(struct resetcause_file_s));
  if (!newattr)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attributes from the old attributes to the new */

  memcpy(newattr, oldattr, sizeof(struct resetcause_file_s));

  /* Save the new attributes in the new file structure */

  newp->f_priv = (FAR void *)newattr;
  return OK;
}

/****************************************************************************
 * Name: resetcause_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int resetcause_stat(FAR const char *relpath, FAR struct stat *buf)
{
  /* "resetcause" is the only acceptable value for the relpath */

  if (strcmp(relpath, "resetcause") != 0)
    {
      ferr("ERROR: relpath is '%s'\n", relpath);
      return -ENOENT;
    }

  /* "resetcause" is the name for a read-only file */

  memset(buf, 0, sizeof(struct stat));
  buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*!
 * @brief   this function registrates the reset cause as a proc fs
 *      
 * @param   none
 *
 * @return  0 if OK, error number otherwise
 *
 */
int s32k1xx_resetcause_initialize_procfs(void)
{
  // int ret;
  return procfs_register(&g_resetcause_procfs);
  // if (ret < 0)
  //     ret = -EPERM;
  // return ret;
}

/*!
 * @brief   this function initializes the resetcause
 *
 *      It will get the resetcause and store it
 *      
 * @param   none
 *
 * @return  none
 *
 */
void s32k1xx_resetcause_init(void)
{
  uint32_t resetCauseRegister = 0;

  // get the reset cause
  resetCauseRegister = getreg32(S32K1XX_RCM_SRS);

  // save it in the global variable 
  gResetCause = (unsigned int) resetCauseRegister;
}

#endif /* CONFIG_RESET_CAUSE_PROC_FS && CONFIG_FS_PROCFS */
