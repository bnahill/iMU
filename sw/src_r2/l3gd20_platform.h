// This file may only be included once

#ifndef __L3GD20_PLATFORM_H_
#define __L3GD20_PLATFORM_H_

#include "spi_platform.h"

/*!
 @file l3gd20_platform.h
 @brief Platform-specific configuration for L3GD20 sensor
 
 This file contains any platform specific declarations for the sensor(s) and
 is to be included only directly in l3gd20.c. Public interfaces should be
 defined in l3gd20_public.h.
 */

#include "l3gd20.h"
#include "l3gd20_private.h"
#include "spi_public.h"

class L3GD20 : public L3GD20_HW<SPI_4Wire> {
public:
	L3GD20(SPI_4Wire &_spi, const gpio_pin_t &_nss) : 
		L3GD20_HW< SPI_4Wire >(spi,nss){}
};

/*
l3gd20_t gyro1 = {
	.reading = {0.0, 0.0, 0.0},
	.nss = {GPIOA, BIT(8)},
	.dps_scale = GYRO_SCALE_500_DPS,
	.spi = &spi1
};
*/

extern L3GD20 gyro1;

#endif

#ifdef __L3GD20_CPP_
L3GD20 gyro1(spi1, {GPIOA, BIT(8)});
#endif

