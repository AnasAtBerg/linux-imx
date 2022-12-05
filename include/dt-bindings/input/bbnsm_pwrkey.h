/*
 * Copyright (C) 2022 Avnet Embedded GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _DT_BINDINGS_BBNSM_PWRKEY_H
#define _DT_BINDINGS_BBNSM_PWRKEY_H

#define BBNSM_TURN_ON_TIME_500MS	(0)
#define BBNSM_TURN_ON_TIME_50MS		(1 << 20)
#define BBNSM_TURN_ON_TIME_100MS	(2 << 20)
#define BBNSM_TURN_ON_TIME_0MS		(3 << 20)

#endif /* _DT_BINDINGS_BBNSM_PWRKEY_H */
