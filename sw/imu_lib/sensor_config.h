#ifndef __SENSOR_CONFIG_H_
#define __SENSOR_CONFIG_H_

#define HAS_GYRO_1 1
#define HAS_GYRO_2 1

#define GYRO_USE_DMA 1

typedef struct {
	float x;
	float y;
	float z;
} euclidean3_t;

#endif

