/******************************************************************************
 * configs/kosagi-fernvale/src/fernvale_spi.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "fernvale_internal.h"
#include "fernvale_h3131.h"

#ifdef CONFIG_FERNVALE_SPI
#if 0 /* At present, LPC-H3131 specific logic is hard-coded in the file fernvale_spi.c
       * in arch/arm/src/fernvalexx */

/******************************************************************************
 * Definitions
 ******************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef SPI_DEBUG   /* Define to enable debug */
#undef SPI_VERBOSE /* Define to enable verbose debug */

#ifdef SPI_DEBUG
#  define spidbg  lldbg
#  ifdef SPI_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef SPI_VERBOSE
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/******************************************************************************
 * Private Functions
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: fernvale_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LPC-H3131 board.
 *
 ******************************************************************************/

void weak_function fernvale_spiinitialize(void)
{
  /* NOTE: Clocking for SPI has already been provided. Pin configuration is performed
   * on-the-fly, so no additional setup is required.
   */
}

/******************************************************************************
 * Name:  fernvale_spiselect and fernvale_spistatus
 *
 * Description:
 *   The external functions, fernvale_spiselect and fernvale_spistatus must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including up_spiinitialize())
 *   are provided by common FERNVALEXX logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in fernvale_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide fernvale_spiselect() and fernvale_spistatus() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ******************************************************************************/

void fernvale_spiselect(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t fernvale_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return SPI_STATUS_PRESENT;
}

#endif /* 0 */
#endif /* CONFIG_FERNVALE_SPI  */

