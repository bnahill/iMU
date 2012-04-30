#ifndef __SENSOR_CONFIG_H_
#define __SENSOR_CONFIG_H_

#ifndef BIT
#define BIT(x) (1 << x)
#endif


//////////////////////////////////////////////////////////////////////////////
// STTS751
//////////////////////////////////////////////////////////////////////////////

#define HAS_STTS 1

//////////////////////////////////////////////////////////////////////////////
// L3GD20
//////////////////////////////////////////////////////////////////////////////

#define HAS_GYRO_1 1
#define HAS_GYRO_2 0

#define GYRO_USE_DMA 1

//////////////////////////////////////////////////////////////////////////////
// LSM303
//////////////////////////////////////////////////////////////////////////////

#define HAS_MAGACC_1 1
#define HAS_MAGACC_2 0

//////////////////////////////////////////////////////////////////////////////
// 
//////////////////////////////////////////////////////////////////////////////


#define USE_I2C1 ((HAS_STTS || HAS_MAGACC_1) ? 1 : 0)
#define USE_I2C2 ((HAS_MAGACC_2) ? 1 : 0)
#define USE_I2C3 0

typedef struct {
	float x;
	float y;
	float z;
} euclidean3_t;

#endif

