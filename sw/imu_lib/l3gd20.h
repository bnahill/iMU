#ifndef __L3GD20_H_
#define __L3DG20_H_

#include "stm32f4xx_conf.h"
#include "spi.h"
#include "sensor_config.h"

typedef struct {
	euclidean3_t        reading;
	float               const dps_scale;
	gpio_pin_t          nss;
	spi_t               * const spi;
	uint8_t             r_buff[7];
	spi_transfer_t      xfer;
} gyro_t;

#if HAS_GYRO_1
extern gyro_t gyro1;
#endif

#if HAS_GYRO_2
extern gyro_t gyro2;
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

