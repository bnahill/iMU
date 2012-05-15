#ifndef __LSM303_H_
#define __LSM303_H_

#include "stm32f4xx_conf.h"
#include "sensor_config.h"
#include "i2c.h"

typedef enum {
	LSM_RATE_OFF  = 0,
	LSM_RATE_1    = 1,
	LSM_RATE_10   = 2,
	LSM_RATE_25   = 3,
	LSM_RATE_50   = 4,
	LSM_RATE_100  = 5,
	LSM_RATE_200  = 6,
	LSM_RATE_400  = 7,
	LSM_RATE_LP16 = 2,
	LSM_RATE_LP53 = 2
} lsm_rate_t;

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

