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
	I2C_ST_REPEAT_ADDR,
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

/*!
 @brief The state and configuration of an I2C device
 */
typedef struct {
	//! The I2C device
	I2C_TypeDef *const i2c;
	//! The mode
	i2c_mode_t mode;
	//! The current operation
	i2c_op_t op;
	//! The current state
	i2c_state_t state;
	//! The I2C address of the slave
	uint8_t devaddr;
	//! The data address to read or write
	uint8_t addr;
	//! The buffer address
	uint8_t *buffer;
	//! The number of bytes to read or write
	uint8_t count;
	//! A flag to indicate completion
	uint8_t done;
	//! A lock for mutual exclusion
	uint32_t lock;
	//! A pointer to the hardware configuration
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

/*!
 @brief Initialize an I2C device
 @param i2c The I2C device to configure
 @param mode The mode to use. Currently only master is implemented
 @param speed The clock speed in Hz
 @return 0 if successful, 1 if already configured (not necessarily a problem)

 Initialize an I2C device. If the device is already initialized, nothing will
 be done and this will return 1. 
 */
int i2c_init(i2c_t *i2c, i2c_mode_t mode, uint32_t speed);

/*!
 @brief Write some bytes to a device over I2C
 @param i2c The I2C device to use
 @param devaddr The address of the slave
 @param addr The address to write to
 @param buffer The start address of the buffer to write
 @param count the number of bytes to write

 Perform a write operation to an I2C slave using the common I2C memory
 interface format. To indicate completion, the caller should specify a

 */
void i2c_write(i2c_t *i2c, uint8_t devaddr, uint8_t addr, uint8_t *buffer, uint8_t count);

#endif

