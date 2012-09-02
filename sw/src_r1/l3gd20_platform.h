// This file may only be included once

#ifndef _L3GD20_C__
#error "l3gd20_platform.h may only be included by l3gd20.c!"
#endif

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

l3gd20_t gyro1 = {
	.reading = {0.0, 0.0, 0.0},
	.nss = {GPIOA, BIT(8)},
	.dps_scale = GYRO_SCALE_500_DPS,
	.spi = &spi1
};

void l3gd20_init(void){
	spi_init_slave(&gyro1.nss);
	l3gd20_device_init(&gyro1);
}

void l3gd20_read(void){
	l3gd20_read_sensor(&gyro1);
}

void l3gd20_update(void){
	l3gd20_update_device(&gyro1);
}

uint8_t l3gd20_transfer_complete(void){
	return gyro1.xfer.done;
}

