#ifndef __I2C_PLATFORM_H_
#define __I2C_PLATFORM_H_

#include <stm32f4xx.h>
#include <i2c.h>


#ifdef __I2C_CPP_

I2C_HWConfig const i2c1_config(I2C1, 100000, GPIO_AF_I2C1,
							   // SDA
							   af_gpio_pin_t(GPIOB, BIT(7), GPIO_PinSource7),
							   // SCL
							   af_gpio_pin_t(GPIOB, BIT(6), GPIO_PinSource6),
							   I2C1_EV_IRQn,
							   I2C1_ER_IRQn,
							   RCC_APB1Periph_I2C1);

I2CMaster_7bDev_8bRegaddrs i2c1(i2c1_config);

extern "C"{
	// G++ really wants to discard these for some reason
	void __attribute__((used)) I2C1_ER_IRQHandler(void){i2c1.isr_error();}
	void __attribute__((used)) I2C1_EV_IRQHandler(void){i2c1.isr_event();}
};

#endif

extern I2CMaster_7bDev_8bRegaddrs i2c1;

#endif
