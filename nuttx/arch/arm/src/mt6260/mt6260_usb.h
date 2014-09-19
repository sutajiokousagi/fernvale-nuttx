/******************************************************************************
 * mt6260/mt6260_usb.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_MT6260_MT6260_USB_H
#define __ARCH_ARM_SRC_MT6260_MT6260_USB_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

// Rx = OUT
// Tx = IN
/* USB Controller Registers ***************************************************/

#define MT6260_USBOTG_VADDR    0xa0900000
#define MT6260_USBOTGPHY_VADDR 0xa0910000

#define MT6260_USB_FADDR       (MT6260_USBOTG_VADDR+0x0000) /* Peripheral Address */
#define MT6260_USB_POWER       (MT6260_USBOTG_VADDR+0x0001) /* Power Control */
#define MT6260_USB_INTRIN      (MT6260_USBOTG_VADDR+0x0002) /* Transmit EP Interrupt Status #1 */
#define MT6260_USB_RSV1        (MT6260_USBOTG_VADDR+0x0003) /* Reserved */
#define MT6260_USB_INTROUT     (MT6260_USBOTG_VADDR+0x0004) /* Receive EP Interrupt Status #1 */
#define MT6260_USB_RSV2        (MT6260_USBOTG_VADDR+0x0005) /* Reserved */
#define MT6260_USB_INTRUSB     (MT6260_USBOTG_VADDR+0x0006) /* USB Interrupt Status */
#define MT6260_USB_INTRINE     (MT6260_USBOTG_VADDR+0x0007) /* Transmit EP Interrupt Enable #1 */
#define MT6260_USB_RSV3        (MT6260_USBOTG_VADDR+0x0008) /* Reserved */
#define MT6260_USB_INTROUTE    (MT6260_USBOTG_VADDR+0x0009) /* Receive EP Interrupt Enable #1 */
#define MT6260_USB_RSV4        (MT6260_USBOTG_VADDR+0x000a) /* Reserved */
#define MT6260_USB_INTRUSBE    (MT6260_USBOTG_VADDR+0x000b) /* USB Interrupt Enable */
#define MT6260_USB_FRAME1      (MT6260_USBOTG_VADDR+0x000c) /* Lower Frame Number */
#define MT6260_USB_FRAME2      (MT6260_USBOTG_VADDR+0x000d) /* Upper Frame Number */
#define MT6260_USB_INDEX       (MT6260_USBOTG_VADDR+0x000e) /* Endpoint Index */
#define MT6260_USB_DEVCTL      (MT6260_USBOTG_VADDR+0x000f) /* Device Control */
#define MT6260_USB_INMAXP      (MT6260_USBOTG_VADDR+0x0010) /* Transmit Maximum Packet Size */
#define MT6260_USB_CSR0        (MT6260_USBOTG_VADDR+0x0011) /* Peripheral EP0 Control */
#define MT6260_USB_INCSR1      (MT6260_USBOTG_VADDR+0x0011) /* Peripheral Tx EP Control #1 */
#define MT6260_USB_INCSR2      (MT6260_USBOTG_VADDR+0x0012) /* Tx EP Control #2 */
#define MT6260_USB_OUTMAXP     (MT6260_USBOTG_VADDR+0x0013) /* Receive Maximum Packet Size */
#define MT6260_USB_OUTCSR1     (MT6260_USBOTG_VADDR+0x0014) /* Peripheral Rx EP Control #1 */
#define MT6260_USB_OUTCSR2     (MT6260_USBOTG_VADDR+0x0015) /* Peripheral Rx EP Control #2 */
#define MT6260_USB_COUNT0      (MT6260_USBOTG_VADDR+0x0016) /* Count EP0 Data Bytes */
#define MT6260_USB_RXCOUNT1    (MT6260_USBOTG_VADDR+0x0016) /* Count EP Data bytes #1 */
#define MT6260_USB_RXCOUNT2    (MT6260_USBOTG_VADDR+0x0017) /* Count EP Data bytes #2 */
#define MT6260_USB_FIFO0       (MT6260_USBOTG_VADDR+0x0020) /* EP0 FIFO Access */
#define MT6260_USB_FIFO1       (MT6260_USBOTG_VADDR+0x0024) /* EP1 FIFO Access */
#define MT6260_USB_FIFO2       (MT6260_USBOTG_VADDR+0x0028) /* EP2 FIFO Access */
#define MT6260_USB_FIFO3       (MT6260_USBOTG_VADDR+0x002c) /* EP3 FIFO Access */
#define MT6260_USB_FIFO4       (MT6260_USBOTG_VADDR+0x0030) /* EP4 FIFO Access */
#define MT6260_USB_RESET       (MT6260_USBOTG_VADDR+0x0230) /* USB Block Reset */
#define MT6260_USB_PHYCTRL     (MT6260_USBOTG_VADDR+0x0240) /* USB PHY Control */
#define MT6260_USB_DUMMY       (MT6260_USBOTG_VADDR+0x0244) /* USB Dummy Register */

#define MT6260_USBPHY_CSR0     (MT6260_USBOTGPHY_VADDR+0x08c0)  /* PHY Control 0 */
#define MT6260_USBPHY_CSR1     (MT6260_USBOTGPHY_VADDR+0x08c4)  /* PHY Control 1 */
#define MT6260_USBPHY_CSR2     (MT6260_USBOTGPHY_VADDR+0x08c8)  /* PHY Control 2 */

/* POWER register bit settings ******************************************************/

#define USB_POWER_ENSUS       (0x01)
#define USB_POWER_SUSPEND     (0x02)
#define USB_POWER_RESUME      (0x04)
#define USB_POWER_RESET       (0x08)
#define USB_POWER_SWRSTENAB   (0x10)
#define USB_POWER_ISO         (0x80)

/* USB interrupt bits **************************************************************/

#define USB_INT_NOINTERRUPT   (0x00000000)
#define USB_INT_SUSPEND       (0x00000001)
#define USB_INT_RESUME        (0x00000002)
#define USB_INT_RESET         (0x00000004)
#define USB_INT_SOF           (0x00000008)
#define USB_INT_POWERDOWN     (0x00000010)
#define USB_INT_OUTFIFO       (0x00000f00)
#define USB_INT_OUTFIFO1      (0x00000100)
#define USB_INT_OUTFIFO2      (0x00000200)
#define USB_INT_OUTFIFO3      (0x00000400)
#define USB_INT_OUTFIFO4      (0x00000800)
#define USB_INT_CONTROL       (0x00001000)
#define USB_INT_INFIFO        (0x0001e000)
#define USB_INT_INFIFO1       (0x00002000)
#define USB_INT_INFIFO2       (0x00004000)
#define USB_INT_INFIFO3       (0x00008000)
#define USB_INT_INFIFO4       (0x00010000)

#define USB_EP4_TX            (0x10)
#define USB_EP3_TX            (0x08)
#define USB_EP2_TX            (0x04)
#define USB_EP1_TX            (0x02)
#define USB_EP0               (0x01)

#define USB_EP2_RX            (0x04)
#define USB_EP1_RX            (0x02)CLRSETEND

/* Endpoint control register index *************************************************/

#define USB_EP0_SELECT        (0x00)

/* Reset Control register bit settings *********************************************/

#define USB_RESET_SWRST         (0x80)
#define USB_RESET_RSTCNTR       (0x00)
#define USB_RESET_RSTCNTR_MASK  (0x0f)

/* CSR0 register bit settings ***************************************************/

#define USB_CSR0_SSETUPEND  (0x80)
#define USB_CSR0_SOUTPKTRDY (0x40)
#define USB_CSR0_SENDSTALL  (0x20)
#define USB_CSR0_SETUPEND   (0x10)
#define USB_CSR0_DATAEND    (0x08)
#define USB_CSR0_SENTSTALL  (0x04)
#define USB_CSR0_INPKTRDY   (0x02)
#define USB_CSR0_OUTPKTRDY  (0x01)

/* INCSR1 register bit settings ****************************************************/

#define USB_INCSR1_CLRDATTOG   (0x40)
#define USB_INCSR1_SENTST      (0x20)
#define USB_INCSR1_SENDST      (0x10)
#define USB_INCSR1_FLFIFO      (0x08)
#define USB_INCSR1_UNDERRUN    (0x04)
#define USB_INCSR1_FIFONOTEMP  (0x02)
#define USB_INCSR1_INPKTRDY    (0x01)

/* INCSR2 register bit settings ****************************************************/

#define USB_INCSR2_AUTOSET     (0x80)
#define USB_INCSR2_ISO         (0x40)
#define USB_INCSR2_MODE_IN     (0x20)
#define USB_INCSR2_DMAEN       (0x10)
#define USB_INCSR2_FRDATTOG    (0x08)
#define USB_INCSR2_DMAMODE1    (0x04)

/* OUTCSR1 register bit settings *************************************************/

#define USB_OUTCSR1_CLRDATTOG (0x80)
#define USB_OUTCSR1_SENTST    (0x40)
#define USB_OUTCSR1_SENDST    (0x20)
#define USB_OUTCSR1_FLFIFO    (0x10)
#define USB_OUTCSR1_DATERR    (0x08)
#define USB_OUTCSR1_OVERRUN   (0x04)
#define USB_OUTCSR1_FIFOFUL   (0x02)
#define USB_OUTCSR1_OUTPKTRDY (0x01)

/* OUTCSR2 register bit settings *************************************************/

#define USB_CSR2_AUTOCLR  (0x80)
#define USB_CSR2_ISO      (0x40)
#define USB_CSR2_DMAEN    (0x20)
#define USB_CSR2_DMAMODE1 (0x10)

/* USB PHY control register bit settings *******************************************/
#define USB_PHY_DPPULLUP       (0x01) /* Enable 1.5k pullup on D+ pin */
#define USB_PHY_DNPULLUP       (0x02) /* Enable 1.5k pullup on D- pin */
#define USB_PHY_TOF            (0x10) /* Time of Flight (?) */
#define USB_PHY_NULLPKT_FIX    (0x20) /* Don't issue a DMA request when a null packet is received */

/* USB Reset control register bit settings *****************************************/

#define USB_RESET_ENTER        (0x00)
#define USB_RESET_EXIT         (0x01)

/* USB PHY Control Status registers ************************************************/

#define USBPHY_CSR0_RESET      (0x00800000)
#define USBPHY_CSR0_FORCE_FSLS (0x00000800)
#define USBPHY_CSR1_REV7       (0x00008000)

/******************************************************************************
 * Inline Functions
 ******************************************************************************/

#endif  /* __ARCH_ARM_SRC_MT6260_MT6260_USB_H */
