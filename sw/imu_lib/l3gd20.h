#ifndef __L3GD20_H_
#define __L3DG20_H_

#include "stm32f4xx_conf.h"
#include "sensor_config.h"

#if HAS_GYRO_1
extern euclidean3_t gyro1_result;
#endif

#if HAS_GYRO_2
extern euclidean3_t gyro2_result;
#endif

void l3gd20_init(void);

void l3gd20_read(void);

#endif

