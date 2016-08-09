/*
 * Definitions for PCM3060 ASoC codec driver
 *
 * Copyright (c) 2010 Michal Bachraty <michal.bachraty@streamunlimited.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PCM3060_H
#define __PCM3060_H

struct pcm3060_platform_data {
	int gpio_nreset;	/* GPIO driving Reset pin, if any */
};

#endif /* __PCM3060_H */
