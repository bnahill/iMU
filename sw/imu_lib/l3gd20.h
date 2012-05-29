#ifndef __L3GD20_H_
#define __L3DG20_H_

#include "stm32f4xx_conf.h"
#include "spi.h"
#include "sensor_config.h"

typedef enum {
	L3GD20_ODR_95 = 0,
	L3GD20_ODR_190 = 1,
	L3GD20_ODR_380 = 2,
	L3GD20_ODR_760 = 3
} l3gd20_odr_t;

typedef enum {
	L3GD20_LP_0 = 0,
	L3GD20_LP_1 = 1,
	L3GD20_LP_2 = 2,
	L3GD20_LP_3 = 3
} l3gd20_lp_cutoff_t;

typedef enum {
	L3GD20_HP_OFF = -1,
	L3GD20_HP_0   = 0,
	L3GD20_HP_1   = 1,
	L3GD20_HP_2   = 2,
	L3GD20_HP_3   = 3
} l3gd20_hp_cutoff_t;

typedef struct {
	euclidean3_t        reading;
	float               const dps_scale;
	gpio_pin_t          nss;
	spi_t               * const spi;
	l3gd20_odr_t        odr;
	l3gd20_lp_cutoff_t  lp_cutoff;
	l3gd20_hp_cutoff_t  hp_cutoff;
	uint8_t             r_buff[7];
	spi_transfer_t      xfer;
} l3gd20_t;

#if HAS_GYRO_1
extern l3gd20_t gyro1;
#endif

#if HAS_GYRO_2
extern l3gd20_t gyro2;
#endif

/*!
 @brief Initialize all attached L3GD20 gyroscopes
 */
void l3gd20_init(void);

/*!
 @brief Start asynchronous sequential reads of all gyroscopes

 Nothing is done upon completion. This function will return very quickly and
 the caller must check l3gd20_transfer_complete to identify when the transfer
 is actually done and then call l3gd20_update to actually update the data
 structures.
 
 @sa l3gd20_transfer_complete()
 @sa l3gd20_update()
 */
void l3gd20_read(void);

/*!
 @brief Update the formatted sensor data fields based on the most recent
 transfer
 */
void l3gd20_update(void);

/*!
 @brief Check to see if the transfer is complete
 @return 0 if incomplete
 */
uint8_t l3gd20_transfer_complete(void);

/*!
 @brief Perform a complete synchronous read of the sensors

 This performs the procedure described in l3gd20_read
 */
void l3gd20_read_sync(void);

#endif

