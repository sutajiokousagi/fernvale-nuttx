/* main program of Free Software for Calypso Phone */

/* (C) 2010 by Harald Welte <laforge@gnumonks.org>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <compat.h>

#include <stdint.h>
#include <stdio.h>

#include <string.h>
#include <rffe.h>
#include <board.h>

#include <abb/twl3025.h>
#include <rf/trf6151.h>

#include <calypso/clock.h>
#include <calypso/tpu.h>
#include <calypso/tsp.h>
#include <calypso/irq.h>
#include <calypso/misc.h>
#include <calypso/sim.h>

#include <layer1/sync.h>
#include <layer1/async.h>
#include <layer1/tpu_window.h>
#include <layer1/l23_api.h>

const char *hr = "======================================================================\n";

void board_io_init(void);

/* MAIN program **************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int layer1_main(int argc, char *argv[])
#endif
{
	uint8_t atr[20];
	uint8_t atrLength = 0;

	board_io_init();

	/* Initialize DMA controller */
	dma_init();

	/* Initialize ABB driver (uses SPI) */
	twl3025_init();

	puts("\n\nOsmocomBB Layer 1 (Nuttx version)\n");
	puts(hr);

	/* Dump device identification */
	dump_dev_id();
	puts(hr);

	/* Dump clock config after PLL set */
	calypso_clk_dump();
	puts(hr);

#if 0
	/* initialize SIM */
	calypso_sim_init();
#endif
	puts("Power up simcard:\n");
	memset(atr,0,sizeof(atr));
#if 0
	atrLength = calypso_sim_powerup(atr);

	layer1_init();

	tpu_frame_irq_en(1, 1);

	while (1) {
		l1a_compl_execute();
		osmo_timers_update();
		sim_handler();
		l1a_l23_handler();
	}

	/* NOT REACHED */

	twl3025_power_off();
#endif

	return 0;
}

static int afcout = 0;

static void tspact_toggle(uint8_t num)
{
	printf("TSPACT%u toggle\n", num);
	tsp_act_toggle((1 << num));
	tpu_enq_sleep();
	tpu_enable(1);
	tpu_wait_idle();
}
