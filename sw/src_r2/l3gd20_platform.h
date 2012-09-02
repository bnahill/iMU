// This file may only be included once

#ifndef _L3GD20_C__
#error "l3gd20_platform.h may only be included by l3gd20.c!"


/*!
 @file l3gd20_platform.h
 @brief Platform-specific configuration for L3GD20 sensor
 
 This file contains any platform specific declarations for the sensor(s) and
 is to be included only directly in l3gd20.c. Public interfaces should be
 defined in l3gd20_public.h.
 */

#include "l3gd20.h"
#include "l3gd20_private.h"


#if HAS_GYRO_1
l3gd20_t gyro1 = {
	.reading = {0.0, 0.0, 0.0},
	.nss = {GPIOC, BIT(1)},
	.dps_scale = GYRO_SCALE_500_DPS,
	.spi = &spi1
};
#endif

#if HAS_GYRO_2
l3gd20_t gyro2 = {
	.reading = {0.0, 0.0, 0.0},
	.nss = {GPIOE, BIT(15)},
	.dps_scale = GYRO_SCALE_500_DPS,
	.spi = &spi1
};
#endif


void l3gd20_init(void){
#if HAS_GYRO_1
	spi_init_slave(&gyro1.nss);
#endif
#if HAS_GYRO_2
	spi_init_slave(&gyro2.nss);
#endif
#if HAS_GYRO_1
	l3gd20_device_init(&gyro1);
#endif
#if HAS_GYRO_2
	l3gd20_device_init(&gyro2);
#endif
}

void l3gd20_read(void){
#if HAS_GYRO_1
	l3gd20_read_sensor(&gyro1);
#endif
#if HAS_GYRO_2
	l3gd20_read_sensor(&gyro2);
#endif
}

void l3gd20_update(void){
#if HAS_GYRO_1
	l3gd20_update_device(&gyro1);
#endif
#if HAS_GYRO_2
	l3gd20_update_device(&gyro2);
#endif
}

uint8_t l3gd20_transfer_complete(void){
	uint8_t ret = 1;
	
	#if HAS_GYRO_1
		ret &= gyro1.xfer.done;
	#endif
	#if HAS_GYRO_2
		ret &= gyro2.xfer.done;
	#endif
	return ret;
}


#endif
