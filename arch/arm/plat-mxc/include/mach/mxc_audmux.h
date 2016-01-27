/*
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __ASM_ARCH_MXC_AUDMUX_V3_H
#define __ASM_ARCH_MXC_AUDMUX_V3_H

#define MXC_AUDMUX_MODE_SSI		0
#define MXC_AUDMUX_MODE_AC97		1
#define MXC_AUDMUX_MODE_I2S_MASTER	2
#define MXC_AUDMUX_MODE_I2S_SLAVE	4

extern int mxc_audmux_v3_configure_sync_slave(unsigned int unit, unsigned int pin,
					unsigned int flags);
extern int mx2_audmux_configure_sync_slave(unsigned int unit, unsigned int pin,
					unsigned int flags);

static inline int mxc_audmux_configure_sync_slave(unsigned int unit,
						unsigned int pin,
						unsigned int flags)
{
#if defined(CONFIG_ARCH_MX2)
	if (cpu_is_mx2())
		return mx2_audmux_configure_sync_slave(unit, pin, flags);
#endif
#if defined(CONFIG_ARCH_MX25)
	if (cpu_is_mx25())
		return mxc_audmux_v3_configure_sync_slave(unit, pin, flags);
#endif
#if defined(CONFIG_ARCH_MX3)
	if (cpu_is_mx3x())
		return mxc_audmux_v3_configure_sync_slave(unit, pin, flags);
#endif
#if defined(CONFIG_ARCH_MX5)
	if (cpu_is_mx5())
		return mxc_audmux_v3_configure_sync_slave(unit, pin, flags);
#endif
	BUG();
}

#endif /* __ASM_ARCH_MXC_AUDMUX_V3_H */
