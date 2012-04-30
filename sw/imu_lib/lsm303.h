#ifndef __LSM303_H_
#define __LSM303_H_

#include "stm32f4xx_conf.h"
#include "sensor_config.h"
#include "i2c.h"

typedef struct {
	euclidean3_t mag;
	euclidean3_t acc;
	i2c_t *const i2c;
} lsm303_t;

#if HAS_MAGACC_1
extern lsm303_t magacc1;
#endif

#if HAS_MAGACC_2
extern lsm303_t magacc2;
#endif

void lsm303_init(void);

#endif

