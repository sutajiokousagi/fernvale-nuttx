/****************************************************************************
 * arch/arm/src/mt6260/mt6260_serial.c
 * arch/arm/src/chip/mt6260_serial.c
 *
 *   Copyright (C) 2007-2009, 2012-2013 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <arch/serial.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t uartbase;	/* Base address of UART registers */
  uint32_t baud;		/* Configured baud */
  uint8_t  msr;			/* Saved MSR value */
  uint8_t  ier;			/* Saved Interrupt Enable Register value */
  uint8_t  lcr;			/* Saved Line Control Register value */
  uint8_t  irq;			/* IRQ associated with this UART */
  uint8_t  parity;		/* 0=none, 1=odd, 2=even */
  uint8_t  bits;		/* Number of bits (7 or 8) */
  bool     stopbits2;	/* true: Configure with 2
						 * stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* I/O buffers */

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

/* This describes the state of the MT6260 uart0 port. */

static struct up_dev_s g_uart0priv =
{
  .uartbase       = MT6260_UART0_REGISTER_BASE,
  .baud           = CONFIG_UART0_BAUD,
  .irq            = MT6260_IRQ_UART0,
  .parity         = CONFIG_UART0_PARITY,
  .bits           = CONFIG_UART0_BITS,
  .stopbits2      = CONFIG_UART0_2STOP,
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};

/* This describes the state of the MT6260 uart1 port. */

static struct up_dev_s g_uart1priv =
{
  .uartbase       = MT6260_UART1_REGISTER_BASE,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = MT6260_IRQ_UART1,
  .parity         = CONFIG_UART1_PARITY,
  .bits           = CONFIG_UART1_BITS,
  .stopbits2      = CONFIG_UART1_2STOP,
};

static uart_dev_t g_uart1port =
{
  .recv     =
  {
    .size   = CONFIG_UART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_uart1priv,
};

/* Now, which one with be tty0/console and which tty1? */

#ifdef CONFIG_SERIAL_IRDA_CONSOLE
# define CONSOLE_DEV     g_uart1port
# define TTYS0_DEV       g_uart1port
# define TTYS1_DEV       g_uart0port
#else
# define CONSOLE_DEV     g_uart0port
# define TTYS0_DEV       g_uart0port
# define TTYS1_DEV       g_uart1port
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint8_t up_serialin(struct up_dev_s *priv, uint32_t offset)
{
  return getreg8(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, uint32_t offset, uint8_t value)
{
  putreg8(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint8_t *ier)
{
  if (ier)
    {
      *ier = priv->ier & UART_IER_ALLIE;
    }

  priv->ier &= ~UART_IER_ALLIE;
  up_serialout(priv, UART_IER, priv->ier);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint8_t ier)
{
  priv->ier |= ier & UART_IER_ALLIE;
  up_serialout(priv, UART_IER, priv->ier);
}

/****************************************************************************
 * Name: up_waittxready
 ****************************************************************************/

static inline void up_waittxready(struct up_dev_s *priv)
{
  int tmp;

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      if ((up_serialin(priv, UART_LSR) & UART_LSR_TEMT) != 0)
        {
          break;
        }
    }
}

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(struct up_dev_s *priv, bool enable)
{
#if 0
  uint8_t lcr = up_serialin(priv, UART_LCR);
  if (enable)
    {
      lcr |= UART_LCR_BOC;
    }
  else
    {
      lcr &= ~UART_LCR_BOC;
    }
  up_serialout(priv, UART_LCR, lcr);
#endif
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Clear fifos, and st up rx and tx triggers */
  up_serialout(priv, UART_FCR, UART_FCR_TFT_1BYTE | UART_FCR_RFT_1BYTE |
                               UART_FCR_CLRT | UART_FCR_CLRR | UART_FCR_FIFOE);

  /* Set up the MSR */

  priv->lcr = 0;
  if (priv->bits == 5)
    {
      priv->lcr |= UART_LCR_WLS_5BIT;
    }
  else if (priv->bits == 6)
    {
      priv->lcr |= UART_LCR_WLS_6BIT;
    }
  else if (priv->bits == 7)
    {
      priv->lcr |= UART_LCR_WLS_7BIT;
    }
  else
    {
      priv->lcr |= UART_LCR_WLS_8BIT;
    }

  if (priv->stopbits2)
    {
      priv->lcr |= UART_LCR_STB_2BITS;
    }
  else
    {
      priv->lcr &= ~UART_LCR_STB_2BITS;
    }

  if (priv->parity == 1)
    {
      priv->lcr |= UART_LCR_STB_PEN;
    }
  else if (priv->parity == 2)
    {
      priv->lcr |= UART_LCR_STB_PEN | UART_LCR_EPS;
    }
  else
    {
      priv->lcr &= ~(UART_LCR_STB_PEN | UART_LCR_EPS);
    }

#if 0
  /* Set up the BRSR */

  switch (priv->baud)
    {
      case 2400:
        brsr = UART_BAUD_2400;
        break;
      case 4800:
        brsr = UART_BAUD_4800;
        break;
      default:
      case 9600:
        brsr = UART_BAUD_9600;
        break;
      case 14400:
        brsr = UART_BAUD_14400;
        break;
      case 19200:
        brsr = UART_BAUD_19200;
        break;
      case 28800:
        brsr = UART_BAUD_28800;
        break;
      case 3840:
        brsr = UART_BAUD_38400;
        break;
      case 57600:
        brsr = UART_BAUD_57600;
        break;
      case 115200:
        brsr = UART_BAUD_115200;
        break;
      case 230400:
        brsr = UART_BAUD_230400;
        break;
      case 460800:
        brsr = UART_BAUD_460800;
        break;
      case 921600:
        brsr = UART_BAUD_921600;
        break;
    }

  /* Setup the new UART configuration */
  up_serialout(priv, UART_BRSR, brsr);
#endif
  up_serialout(priv, UART_LCR, priv->lcr);
  up_enablebreaks(priv, false);
#endif
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt);
  if (ret == OK)
    {
       /* Enable the interrupt (RX and TX interrupts are still disabled
        * in the UART
        */

       up_enable_irq(priv->irq);
    }
  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked
 *   when an interrupt received on the 'irq'  It should call
 *   uart_transmitchars or uart_receivechar to perform the
 *   appropriate data transfers.  The interrupt handling logic\
 *   must be able to map the 'irq' number into the approprite
 *   uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;
  struct up_dev_s   *priv;
  uint32_t           status;
  uint32_t           old_lsr;
  int                passes = 0;

  if (g_uart1priv.irq == irq)
    {
      dev = &g_uart1port;
    }
  else if (g_uart0priv.irq == irq)
    {
      dev = &g_uart0port;
    }
  else
    {
      PANIC();
    }
  priv = (struct up_dev_s*)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (;;)
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

      status  = up_serialin(priv, UART_IIR) & UART_IIR_ID_MASK;

      if (status == UART_IIR_ID_NONE || passes > 256)
        {
          return OK;
        }

      /* Handline incoming, receive bytes */

      else if (status == UART_IIR_ID_RXAVAIL)
        {
          uart_recvchars(dev);
        }

      /* Handle outgoing, transmit bytes */

      else if (status == UART_IIR_ID_TXEMPTY)
        {
          uart_xmitchars(dev);
        }

      else
        {
          lowsyslog("ISR: Unrecognized status: 0x%x\n", status);
        }

      /* Keep track of how many times we do this in case there
       * is some hardware failure condition.
       */

      passes++;
    }
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  struct up_dev_s   *priv  = (struct up_dev_s*)dev->priv;
  int                ret    = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct up_dev_s *user = (struct up_dev_s*)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct up_dev_s));
           }
       }
       break;
#endif

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = irqsave();
        up_enablebreaks(priv, true);
        irqrestore(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = irqsave();
        up_enablebreaks(priv, false);
        irqrestore(flags);
      }
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint8_t byte;

  byte    = up_serialin(priv, UART_RBR);
  *status = 0;
  return byte;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_IER_ERBFI;
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_ERBFI;
    }
  up_serialout(priv, UART_IER, priv->ier);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, UART_LSR) & UART_LSR_DR) != 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_serialout(priv, UART_THR, (uint8_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_IER_ETBEI;
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_ETBEI;
    }
  up_serialout(priv, UART_IER, priv->ier);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, UART_LSR) & UART_LSR_THRE) != 0);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, UART_LSR) & UART_LSR_TEMT) != 0);
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before up_serialinit.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  up_disableuartint(TTYS0_DEV.priv, NULL);
  up_disableuartint(TTYS1_DEV.priv, NULL);

  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
  (void)uart_register("/dev/console", &CONSOLE_DEV);
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
  struct up_dev_s *priv = (struct up_dev_s*)CONSOLE_DEV.priv;
  uint8_t  ier;

  up_disableuartint(priv, &ier);
  up_waittxready(priv);
  up_serialout(priv, UART_THR, (uint8_t)ch);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxready(priv);
      up_serialout(priv, UART_THR, '\r');
    }

  up_waittxready(priv);
  up_restoreuartint(priv, ier);
  return ch;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Definitions
 ****************************************************************************/

#  ifdef CONFIG_UART1_SERIAL_CONSOLE
#    define MT6260_REGISTER_BASE MT6260_UART1_REGISTER_BASE
#  else
#    define MT6260_REGISTER_BASE MT6260_UART0_REGISTER_BASE
#  endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void up_waittxready(void)
{
  int tmp;

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {

      if ((getreg8(MT6260_REGISTER_BASE + UART_LSR) & UART_LSR_THRE) != 0)
        {
          break;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int up_putc(int ch)
{
  up_waittxready();
  putreg8((uint8_t)ch, MT6260_REGISTER_BASE + UART_THR);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxready();
      putreg8((uint8_t)'\r', MT6260_REGISTER_BASE + UART_THR);
    }

  up_waittxready();
  return ch;
}

#endif /* USE_SERIALDRIVER */


