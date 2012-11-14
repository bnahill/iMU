#ifndef __SENSOR_CONFIG_H_
#define __SENSOR_CONFIG_H_

#include "toolchain.h"
extern "C" {
	#include "stm32f4xx.h"
}

#ifndef BIT
#define BIT(x) (1 << x)
#endif

#ifndef NULL
#define NULL ((void *) 0)
#endif

#define USE_L3GD20 1

//////////////////////////////////////////////////////////////////////////////
// STTS751
//////////////////////////////////////////////////////////////////////////////

#define HAS_STTS 1

//////////////////////////////////////////////////////////////////////////////
// L3GD20
//////////////////////////////////////////////////////////////////////////////

#define HAS_GYRO_1 1
#define HAS_GYRO_2 1

//////////////////////////////////////////////////////////////////////////////
// LSM303
//////////////////////////////////////////////////////////////////////////////

#define HAS_MAGACC_1 1
#define HAS_MAGACC_2 1

//////////////////////////////////////////////////////////////////////////////
// SD Card
//////////////////////////////////////////////////////////////////////////////

#define USE_SDIO     1

//////////////////////////////////////////////////////////////////////////////
// Doxygen-related extras
//////////////////////////////////////////////////////////////////////////////

#ifdef DOXYGEN
	#undef HAS_STTS
	#undef HAS_GYRO_1
	#undef HAS_GYRO_2
	#undef HAS_MAGACC_1
	#undef HAS_MAGACC_2

	#define HAS_STTS     1
	#define HAS_GYRO_1   1
	#define HAS_GYRO_2   0
	#define HAS_MAGACC_1 1
	#define HAS_MAGACC_2 1
#endif

//////////////////////////////////////////////////////////////////////////////
// 
//////////////////////////////////////////////////////////////////////////////

#define USE_I2C1 ((HAS_STTS || HAS_MAGACC_1) ? 1 : 0)
#define USE_I2C2 ((HAS_MAGACC_2) ? 1 : 0)
#define USE_I2C3 0

#define USE_SPI1 ((HAS_GYRO_1 || HAS_GYRO_2) ? 1 : 0)
#define USE_SPI2 0

#if USE_I2C3 && USE_SDIO
	#error "SDIO and I2C3 cannot be used at the same time"
#endif

typedef struct af_gpio_pin {
	af_gpio_pin(GPIO_TypeDef *_gpio, uint16_t _pin, uint8_t _pinsrc) :
		gpio(_gpio), pin(_pin), pinsrc(_pinsrc){}
	GPIO_TypeDef *const gpio;
	uint16_t const pin;
	uint8_t const pinsrc;
} af_gpio_pin_t;

typedef struct gpio_pin {
	gpio_pin(GPIO_TypeDef *_gpio, uint16_t _pin) :
		gpio(_gpio), pin(_pin){}
	GPIO_TypeDef *const gpio;
	uint16_t const pin;
} gpio_pin_t;

typedef struct {
	float x;
	float y;
	float z;
} euclidean3_t;


#endif

