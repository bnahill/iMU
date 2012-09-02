#ifndef __ADXL345_H_
#define __ADXL345_H_

#include "stm32f4xx_conf.h"
#include "spi.h"
#include "sensor_config.h"

typedef struct {
	spi_t * const   spi;
	gpio_pin_t      nss;
} adxl345_t;

void adxl345_init(adxl345_t *__restrict acc);

#endif
