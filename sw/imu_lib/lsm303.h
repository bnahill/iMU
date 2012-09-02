#ifndef __LSM303_H_
#define __LSM303_H_

#include "sensor_config.h"
#if USE_LSM303DLHC

#include "stm32f4xx_conf.h"
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

typedef enum {
	LSM_ACC_FS_2G = 0,
	LSM_ACC_FS_4G = 1,
	LSM_ACC_FS_8G = 2,
	LSM_ACC_FS_16G = 3
} lsm_acc_fs_t;

typedef enum {
	LSM_MAG_FS_1_3 = 1,
	LSM_MAG_FS_1_9 = 2,
	LSM_MAG_FS_2_5 = 3,
	LSM_MAG_FS_4_0 = 4,
	LSM_MAG_FS_4_7 = 5,
	LSM_MAG_FS_5_6 = 6,
	LSM_MAG_FS_8_1 = 7
} lsm_mag_fs_t;

typedef struct {
	euclidean3_t mag;
	euclidean3_t acc;
	i2c_t *const i2c;
	lsm_rate_t   rate;
	lsm_acc_fs_t acc_fs;
	lsm_mag_fs_t mag_fs;
} lsm303_t;

#if HAS_MAGACC_1
extern lsm303_t magacc1;
#endif

#if HAS_MAGACC_2
extern lsm303_t magacc2;
#endif

void lsm303_init(void);
void lsm303_read(void);

#endif // USE_LSM303DLHC

#endif

