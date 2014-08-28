/******************************************************************************
 * mt6260/mt6260_uart.h
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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

#ifndef __MT6260_UART_H
#define __MT6260_UART_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* UART definitions ***********************************************************/

/* UART Registers (offsets from the register base) */

#define UART_RBR              0   /* RX Buffer Register */
#define UART_THR              0   /* TX Holding Register */
#define UART_IER              4   /* Interrupt Enable Register */
#define UART_IIR              8   /* Interrupt Identification Register */
#define UART_FCR              8   /* FIFO Control Register */
#define UART_LCR              12  /* Line Control Register */
#define UART_MCR              16  /* Modem Control Register */
#define UART_LSR              20  /* Line Status Register */
#define UART_MSR              24  /* Modem Status Register */
#define UART_SCR              28  /* Scratch register */
#define UART_AUTOBAUD_EN      32  /* Auto Baud Detect Enable Register */
#define UART_HIGHSPEED        36  /* High Speed Mode Register */
#define UART_SAMPLE_COUNT     40  /* Sample Counter Register */
#define UART_SAMPLE_POINT     44  /* Sample Point Register */
#define UART_AUTOBAUD_REG     48  /* Auto Baud Monitor Register */
#define UART_RATEFIX_AD       52  /* Clock Rate Fix Register */
#define UART_AUTOBAUDSAMPLE   56  /* Auto Baud Sample Register */
#define UART_GUARD            60  /* Guard Time Added Register */
#define UART_ESCAPE_DAT       64  /* Escape Char Register */
#define UART_ESCAPE_EN        68  /* Escape Enable Register */
#define UART_SLEEP_EN         72  /* Sleep Enable Register */
#define UART_DMA_EN           76  /* DMA Enable Register */
#define UART_RXTRI_AD         80  /* Rx Trigger Register */
#define UART_FRACDIV_L        84  /* Fractional Divider LSB Register */
#define UART_FRACDIV_M        88  /* Fractional Divider MSB Register */
#define UART_FCR_RD           92  /* FIFO Control Read Out Register */

/* Available when UART_LCR is set to 0xbf */
#define UART_DLL              0   /* Divisor Latch (LS) Register */
#define UART_DLH              4   /* Divisor Latch (MS) Register */
#define UART_EFR              8   /* Enhanced Feature Register */
#define UART_XON1             12  /* XON1 Char Register */
#define UART_XON2             16  /* XON2 Char Register */
#define UART_XOFF1            20  /* XOFF1 Char Register */
#define UART_XOFF2            24  /* XOFF2 Char Register */

#define UART_IER_ERBFI        (1 << 0)  /* Enable RX Buffer Full IRQ */
#define UART_IER_ETBEI        (1 << 1)  /* Enable TX Buffer Full IRQ */
#define UART_IER_ELSI         (1 << 2)  /* Enable LSR Status Change IRQ */
#define UART_IER_EDSSI        (1 << 3)  /* Enable DDSR/DCTS IRQ */
#define UART_IER_VFF_FC_EN    (1 << 4)  /* Enable Flow Control IRQ */
#define UART_IER_XOFFI        (1 << 5)  /* Enable XOFF Flow Control IRQ */
#define UART_IER_RTSI         (1 << 6)  /* Enable Modem RTS IRQ */
#define UART_IER_CTSI         (1 << 7)  /* Enable Modem CTS IRQ */
#define UART_IER_ALLIE        (UART_IER_ERBFI | UART_IER_ETBEI | \
                               UART_IER_ELSI  | UART_IER_EDSSI)

#define UART_IIR_ID_MASK      0x3f
#define UART_IIR_FIFOE_MASK   0xc0

#define UART_IIR_ID_MSR       0x00  /* DDSR or DCTS set in MSR */
#define UART_IIR_ID_NONE      0x01  /* No IRQs pending */
#define UART_IIR_ID_TXEMPTY   0x02  /* TX empty, or TX FIFO trigger level reached */
#define UART_IIR_ID_RXAVAIL   0x04  /* RX available, or RX FIFO trigger level reached */
#define UART_IIR_ID_LSR       0x06  /* BI, FE, PE, or OE set in LSR */
#define UART_IIR_ID_RXTIMEOUT 0x0c  /* Time out on character in RX FIFO */
#define UART_IIR_ID_XOFF      0x10  /* XOFF character received */
#define UART_IIR_ID_FLOW      0x20  /* Hardware flow control rising edge */

#define UART_FCR_FIFOE        (1 << 0)  /* FIFO Enable */
#define UART_FCR_CLRR         (1 << 1)  /* Clear RX FIFO */
#define UART_FCR_CLRT         (1 << 2)  /* Clear TX FIFO */
#define UART_FCR_TFT          (1 << 4)  /* TX FIFO Trigger threshold */
#define UART_FCR_TFT_1BYTE    (0 << 4)  /* TX FIFO Trigger threshold on one byte */
#define UART_FCR_TFT_4BYTES   (1 << 4)  /* TX FIFO Trigger threshold on four bytes */
#define UART_FCR_TFT_8BYTES   (2 << 4)  /* TX FIFO Trigger threshold on eight bytes */
#define UART_FCR_TFT_14BYTES  (3 << 4)  /* TX FIFO Trigger threshold on fourteen bytes */
#define UART_FCR_RFT          (1 << 6)  /* RX FIFO Trigger threshold */
#define UART_FCR_RFT_1BYTE    (0 << 6)  /* RX FIFO Trigger threshold on one byte */
#define UART_FCR_RFT_6BYTES   (1 << 6)  /* RX FIFO Trigger threshold on six bytes */
#define UART_FCR_RFT_12BYTES  (2 << 6)  /* RX FIFO Trigger threshold on twelve bytes */
#define UART_FCR_RFT_RXTRIG   (3 << 6)  /* RX FIFO Trigger threshold on [RXTRIG] bytes */

#define UART_EFR_ENABLE_E     (1 << 4)  /* Enable enhancement features */
#define UART_EFR_LOWER_RATEFIX (1 << 5) /* Enable lower-clock ratefix */
#define UART_EFR_AUTO_RTS     (1 << 6)  /* Enable hardware rx flow ctrl */
#define UART_EFR_AUTO_CTX     (1 << 7)  /* Enable hardware tx flow ctrl */

#define UART_LCR_WLS_5BIT     (0 << 0)  /* Word length (5 bits) */
#define UART_LCR_WLS_6BIT     (1 << 0)  /* Word length (6 bits) */
#define UART_LCR_WLS_7BIT     (2 << 0)  /* Word length (7 bits) */
#define UART_LCR_WLS_8BIT     (3 << 0)  /* Word length (8 bits) */
#define UART_LCR_STB_1BIT     (0 << 2)  /* Stop bits */
#define UART_LCR_STB_2BITS    (1 << 2)  /* Stop bits */
#define UART_LCR_STB_PEN      (1 << 3)  /* Parity enable */
#define UART_LCR_EPS          (1 << 4)  /* Set even parity */
#define UART_LCR_SP           (1 << 5)  /* Stick parity */
#define UART_LCR_SB           (1 << 6)  /* Set break */
#define UART_LCR_DLAB         (1 << 7)  /* Divisor latch access bit */

#define UART_MCR_DTR          (1 << 0)  /* NDTR state */
#define UART_MCR_RTS          (1 << 1)  /* NRTS state */
#define UART_MCR_LOOP         (1 << 4)  /* Loopback control bit */
#define UART_MCR_XOFF_STATUS  (1 << 7)  /* Read-only.  0 when XON received. */

#define UART_LSR_DR           (1 << 0)  /* Data Ready */
#define UART_LSR_OE           (1 << 1)  /* Overrun Error */
#define UART_LSR_PE           (1 << 2)  /* Parity Error */
#define UART_LSR_FE           (1 << 3)  /* Framing Error */
#define UART_LSR_BI           (1 << 4)  /* Break Interrupt */
#define UART_LSR_THRE         (1 << 5)  /* Threshold in TX holding register */
#define UART_LSR_TEMT         (1 << 6)  /* 1 if THR is empty */
#define UART_LSR_FIFOERR      (1 << 7)  /* RX FIFO error indicator */

#define UART_MSR_DCTS         (1 << 0)  /* Delta clear to send */
#define UART_MSR_DDSR         (1 << 1)  /* Delta data set ready */
#define UART_MSR_CTS          (1 << 4)  /* Clear to send */
#define UART_MSR_DSR          (1 << 5)  /* Data set ready */

/******************************************************************************
 * Inline Functions
 ******************************************************************************/

#endif  /* __MT6260_UART_H */
