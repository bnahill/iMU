#ifndef __L3GD20_H_
#define __L3DG20_H_

#include "stm32f4xx_conf.h"
#include "sensor_config.h"

typedef struct {
	euclidean3_t        reading;
	GPIO_TypeDef        *const cs_gpio;
	uint16_t            const cs_pin_mask;
	float               const dps_scale;
} gyro_t;

#if HAS_GYRO_1
extern gyro_t gyro1;
#endif

#if HAS_GYRO_2
extern gyro_t gyro2;
#endif

void l3gd20_init(void);

void l3gd20_read(void);

#endif

