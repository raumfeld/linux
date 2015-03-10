/* rc-rc6-mce.c - Keytable for Philips remotes.
 *
 * Copyright (c) 2010 by Matus Ujhelyi <matus.ujhelyi@streamunlimited.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <media/rc-map.h>
#include <linux/module.h>

static struct rc_map_table rc6_philips[] = {

	/* first remote */
	{ 0x350c, KEY_POWER2 },
	{ 0x043f, KEY_DVD },
	{ 0x356d, KEY_RED },
	{ 0x356e, KEY_GREEN },
	{ 0x356f, KEY_YELLOW },
	{ 0x3570, KEY_BLUE },

	{ 0x352c, KEY_OK },
	{ 0x3558, KEY_UP },
	{ 0x3559, KEY_DOWN },
	{ 0x355a, KEY_LEFT },
	{ 0x355b, KEY_RIGHT },

	{ 0x3520, KEY_NEXT },
	{ 0x3521, KEY_PREVIOUS },

	{ 0x3500, KEY_NUMERIC_0 },
	{ 0x3501, KEY_NUMERIC_1 },
	{ 0x3502, KEY_NUMERIC_2 },
	{ 0x3503, KEY_NUMERIC_3 },
	{ 0x3504, KEY_NUMERIC_4 },
	{ 0x3505, KEY_NUMERIC_5 },
	{ 0x3506, KEY_NUMERIC_6 },
	{ 0x3507, KEY_NUMERIC_7 },
	{ 0x3508, KEY_NUMERIC_8 },
	{ 0x3509, KEY_NUMERIC_9 },

	{ 0x3530, KEY_PLAYPAUSE },
	{ 0x3531, KEY_STOP },
	{ 0x3583, KEY_ESC },

	/* second remote ("HDD & DVD recorder") */
	{ 0x326d, KEY_RED },
	{ 0x326e, KEY_GREEN },
	{ 0x326f, KEY_YELLOW },
	{ 0x3270, KEY_BLUE },

	{ 0x325c, KEY_OK },
	{ 0x3258, KEY_UP },
	{ 0x3259, KEY_DOWN },
	{ 0x325a, KEY_LEFT },
	{ 0x325b, KEY_RIGHT },

	{ 0x3220, KEY_NEXT },
	{ 0x3221, KEY_PREVIOUS },

	{ 0x3200, KEY_NUMERIC_0 },
	{ 0x3201, KEY_NUMERIC_1 },
	{ 0x3202, KEY_NUMERIC_2 },
	{ 0x3203, KEY_NUMERIC_3 },
	{ 0x3204, KEY_NUMERIC_4 },
	{ 0x3205, KEY_NUMERIC_5 },
	{ 0x3206, KEY_NUMERIC_6 },
	{ 0x3207, KEY_NUMERIC_7 },
	{ 0x3208, KEY_NUMERIC_8 },
	{ 0x3209, KEY_NUMERIC_9 },

	{ 0x322c, KEY_PLAYPAUSE },
	{ 0x3231, KEY_STOP },
	{ 0x3283, KEY_ESC }, /* "back" */

	{ 0x32f1, KEY_CONTEXT_MENU }, /* "edit" */
};

static struct rc_map_list rc6_philips_map = {
	.map = {
		.scan    = rc6_philips,
		.size    = ARRAY_SIZE(rc6_philips),
		.rc_type = RC_TYPE_RC6_0,
		.name    = RC_MAP_RC6_PHILIPS,
	}
};

static int __init init_rc_map_rc6_philips(void)
{
	return rc_map_register(&rc6_philips_map);
}

static void __exit exit_rc_map_rc6_philips(void)
{
	rc_map_unregister(&rc6_philips_map);
}

module_init(init_rc_map_rc6_philips)
module_exit(exit_rc_map_rc6_philips)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matus Ujhelyi <matus.ujhelyi@streamunlimited.com>");
