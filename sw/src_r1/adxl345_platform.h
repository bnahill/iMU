// This file may only be included once

#ifndef _ADXL345_C__
#error "adxl345_platform.h may only be included by adxl345.c!"
#endif

/*!
 @file adxl345_platform.h
 @brief Platform-specific configuration for ADXL345 sensor
 
 */

#include "adxl345.h"
//#include "l3gd20_private.h"
#include "spi_public.h"

adxl345_t acc1 = {
	.spi = &spi1,
	.nss = {GPIOC, BIT(5)}
};

