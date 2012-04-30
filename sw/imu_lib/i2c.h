#ifndef __I2C_H_
#define __I2C_H_

#include "sensor_config.h"

typedef enum {
	I2C_MODE_DISABLED = 0,
	I2C_MODE_MASTER,
	//! Not implemented
	I2C_MODE_SLAVE
} i2c_mode_t;

typedef enum {
	I2C_OP_READ,
	I2C_OP_WRITE
} i2c_op_t;

typedef enum {
	I2C_ST_IDLE,
	I2C_ST_MASTER_REQ,
	I2C_ST_ADDRESSED,
	I2C_ST_ADDR_TXED,
	I2C_ST_REPEAT_START,
	I2C_ST_READING,
	I2C_ST_WRITING,
	I2C_ST_CLOSING_WRITE
} i2c_state_t;

/*!
 @brief A set of hardware-related constants for each I2C device
 */
typedef struct {
	uint8_t const af;
	GPIO_TypeDef *const sda_gpio;
	uint16_t const sda_pin;
	uint8_t const sda_pinsrc;
	GPIO_TypeDef *const scl_gpio;
	uint16_t const scl_pin;
	uint8_t const scl_pinsrc;
	IRQn_Type const irq_ev;
	IRQn_Type const irq_er;
	uint32_t const clock;
} i2c_config_t;

typedef struct {
	I2C_TypeDef *const i2c;
	i2c_mode_t mode;
	i2c_op_t op;
	i2c_state_t state;
	uint8_t devaddr;
	uint8_t addr;
	uint8_t *buffer;
	uint8_t count;
	uint8_t *flag;
	uint32_t lock;
	i2c_config_t const * const config;
} i2c_t;

#if USE_I2C1
extern i2c_t i2c1;
#endif

#if USE_I2C2
extern i2c_t i2c2;
#endif

#if USE_I2C3
extern i2c_t i2c3;
#endif

int i2c_init(i2c_t *i2c, i2c_mode_t mode, uint32_t speed);
void i2c_write(i2c_t *i2c, uint8_t devaddr, uint8_t addr, uint8_t *buffer, uint8_t count, int8_t *flag);

#endif

