/*
 * SiS Touchconroller driver for I2C interface
 *
 * Copyright (C) 2015 MSC Technologies
 * Author: Prajosh Premdas <premdas.prajosh@gmail.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef __LINUX_PLATFORM_DATA_SIS_I2C_TS_H
#define __LINUX_PLATFORM_DATA_SIS_I2C_TS_H

/* The platform data for the driver */
struct sis_platform_data {
	unsigned long gpio_int;
	unsigned long gpio_rst;
	const char *input_name;
};

#endif /* __LINUX_PLATFORM_DATA_SIS_I2C_TS_H */

