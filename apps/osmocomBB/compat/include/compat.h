/* (C) 2014 by Denis 'GNUtoo' Carikli <GNUtoo@no-log.org>
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

#ifndef NUTTX_OSMOCOMBB_COMPAT_H
#define NUTTX_OSMOCOMBB_COMPAT_H

#define readb(a) getreg8(a)
#define readw(a) getreg16(a)
#define readl(a) getreg32(a)

#define writeb(v,a) putreg8(v,a)
#define writew(v,a) putreg16(v,a)
#define writel(v,a) putreg32(v,a)

#endif /* NUTTX_OSMOCOMBB_COMPAT_H */
