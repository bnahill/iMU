#ifndef __I2C_H_
#define __I2C_H_

#include "sensor_config.h"

typedef enum {
	I2C_OP_READ,
	I2C_OP_WRITE
} i2c_op_t;

//! @name I2C transfer termination flags
//! @{
#define I2C_XFER_FLAG_DONE 0x01
#define I2C_XFER_FLAG_ERR_MASK 0xFE
#define I2C_XFER_ERR_NOSLAVE 0x02
//! @}


/*!
 @brief A transfer class for devices using 7-bit device addresses and 8-bit
        register addressing
 */
class I2CMaster_7bDev_8bRegaddrs_Xfer {
public:
	I2CMaster_7bDev_8bRegaddrs_Xfer() : next(nullptr), done(0){}
	I2CMaster_7bDev_8bRegaddrs_Xfer(i2c_op_t op, uint8_t devaddr,
	                                uint8_t addr, uint8_t * buffer,
	                                uint8_t count) :
		op(op), devaddr(devaddr), addr(addr), buffer(buffer), count(count),
		// Initial values
		next(nullptr), done(0)
		{}
	
	//! The operation to perform
	i2c_op_t op;
	//! The I2C address of the slave
	uint8_t devaddr;
	//! The data address to read or write
	uint8_t addr;
	//! The buffer address
	uint8_t *buffer;
	//! The number of bytes to read or write
	uint8_t count;
	
	//! A flag to indicate completion and errors
	uint8_t done;
	I2CMaster_7bDev_8bRegaddrs_Xfer * next;
};

/*!
 @brief A set of hardware-related constants for each I2C device
 */
class I2C_HWConfig {
public:
	I2C_HWConfig(I2C_TypeDef *i2c, uint32_t speed, uint8_t af,
	             af_gpio_pin_t sda, af_gpio_pin_t scl,
	             IRQn_Type irq_ev, IRQn_Type irq_er,
	             uint32_t clock) :
		i2c(i2c), speed(speed), af(af), sda(sda), scl(scl),
		irq_ev(irq_ev), irq_er(irq_er), clock(clock){}
	I2C_TypeDef    *i2c;
	uint32_t       speed;
	uint8_t        af;
	af_gpio_pin_t  sda;
	af_gpio_pin_t  scl;
	IRQn_Type      irq_ev;
	IRQn_Type      irq_er;
	uint32_t       clock;
};

/*!
 @brief A template class for I2C master devices
 @tparam transfertype The datatype to use for a single transfer
 
 Variability is welcome for different addressing mechanisms.
 */
template <class transfertype>
class I2CMaster {
public:
	I2CMaster(I2C_HWConfig const &_config) :
		config(_config), xfer(nullptr), is_init(false){}
	
	/*!
	 @brief Test to make sure the signal wires seem to be working properly
	 @return True if successful
	 @post Both pins will be inputs
	 */
	bool test_wires(){
		GPIO_InitTypeDef gpio_init_s;

		GPIO_StructInit(&gpio_init_s);

		// Mode configuration
		gpio_init_s.GPIO_Mode = GPIO_Mode_OUT;
		gpio_init_s.GPIO_Speed = GPIO_Speed_100MHz;
		gpio_init_s.GPIO_PuPd = GPIO_PuPd_NOPULL;
		gpio_init_s.GPIO_OType = GPIO_OType_OD;

		config.sda.gpio->BSRRL = config.sda.pin;
		config.scl.gpio->BSRRL = config.scl.pin;
		
		gpio_init_s.GPIO_Pin = config.sda.pin;
		GPIO_Init(config.sda.gpio, &gpio_init_s);
		gpio_init_s.GPIO_Pin = config.scl.pin;
		GPIO_Init(config.scl.gpio, &gpio_init_s);
		
		if(config.sda.gpio->IDR & config.sda.pin == 0)
			// SDA must be pulled to ground or without adequate pullups
			return false;
		
		if(config.scl.gpio->IDR & config.scl.pin == 0)
			// SCL must be pulled to ground or without adequate pullups
			return false;
		
		config.sda.gpio->BSRRH = config.sda.pin;
		config.scl.gpio->BSRRH = config.scl.pin;
		
		if(config.sda.gpio->IDR & config.scl.pin == 0)
			// SDA must be pulled high
			return false;
		
		if(config.scl.gpio->IDR & config.scl.pin == 0)
			// SCL must be pulled high
			return false;
		
		// Reconfigure to input
		gpio_init_s.GPIO_Mode = GPIO_Mode_IN;
		gpio_init_s.GPIO_Speed = GPIO_Speed_100MHz;
		gpio_init_s.GPIO_PuPd = GPIO_PuPd_NOPULL;
		gpio_init_s.GPIO_OType = GPIO_OType_OD;
		
		gpio_init_s.GPIO_Pin = config.sda.pin;
		GPIO_Init(config.sda.gpio, &gpio_init_s);
		gpio_init_s.GPIO_Pin = config.scl.pin;
		GPIO_Init(config.scl.gpio, &gpio_init_s);
		
		return true;
	}
	
	bool init(){
		GPIO_InitTypeDef gpio_init_s;
		I2C_InitTypeDef i2c_init_s;
		NVIC_InitTypeDef nvic_init_s;

		__disable_irq();
		
		// Check for prior initialization
		if(is_init){
			__enable_irq();
			return false;
		}

		GPIO_StructInit(&gpio_init_s);
		I2C_StructInit(&i2c_init_s);

		// All I2C peripherals are on APB1
		RCC_APB1PeriphClockCmd(config.clock, ENABLE);
		
		// Reset
		RCC_APB1PeriphResetCmd(config.clock, ENABLE);
		RCC_APB1PeriphResetCmd(config.clock, DISABLE);

		////////////////////////////////////////////////////////////////////
		// GPIO Config
		////////////////////////////////////////////////////////////////////

		// Set alternate functions to use
		GPIO_PinAFConfig(config.sda.gpio, config.sda.pinsrc, config.af);
		GPIO_PinAFConfig(config.scl.gpio, config.scl.pinsrc, config.af);

		// Mode configuration
		gpio_init_s.GPIO_Mode = GPIO_Mode_AF;
		gpio_init_s.GPIO_Speed = GPIO_Speed_100MHz;
		gpio_init_s.GPIO_PuPd = GPIO_PuPd_UP;
		gpio_init_s.GPIO_OType = GPIO_OType_OD;

		// For some reason the order here matters and it really shouldn't
		gpio_init_s.GPIO_Pin = config.scl.pin;
		GPIO_Init(config.scl.gpio, &gpio_init_s);
		gpio_init_s.GPIO_Pin = config.sda.pin;
		GPIO_Init(config.sda.gpio, &gpio_init_s);

		////////////////////////////////////////////////////////////////////
		// I2C Config
		////////////////////////////////////////////////////////////////////

		i2c_init_s.I2C_Mode = I2C_Mode_I2C;
		i2c_init_s.I2C_DutyCycle = I2C_DutyCycle_2;
		i2c_init_s.I2C_ClockSpeed = config.speed;
		i2c_init_s.I2C_OwnAddress1 = 0xA0;
		i2c_init_s.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		i2c_init_s.I2C_Ack = I2C_Ack_Enable;

		////////////////////////////////////////////////////////////////////
		// Interrupt Config
		////////////////////////////////////////////////////////////////////

		nvic_init_s.NVIC_IRQChannel = config.irq_er;
		nvic_init_s.NVIC_IRQChannelSubPriority = 2;
		nvic_init_s.NVIC_IRQChannelPreemptionPriority = 2;
		nvic_init_s.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic_init_s);

		nvic_init_s.NVIC_IRQChannel = config.irq_ev;
		nvic_init_s.NVIC_IRQChannelSubPriority = 3;
		nvic_init_s.NVIC_IRQChannelPreemptionPriority = 3;
		nvic_init_s.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic_init_s);

		I2C_ITConfig(config.i2c, I2C_IT_ERR, ENABLE);
		I2C_ITConfig(config.i2c, I2C_IT_BUF, ENABLE);
		I2C_ITConfig(config.i2c, I2C_IT_EVT, ENABLE);

		////////////////////////////////////////////////////////////////////
		// Finalize
		////////////////////////////////////////////////////////////////////

		I2C_Cmd(config.i2c, ENABLE);

		I2C_Init(config.i2c, &i2c_init_s);
			
		is_init = true;
		
		__enable_irq();
		return true;
	}
	
	bool is_init;
	
	void transfer(transfertype * __restrict new_xfer){
		transfertype * old_xfer;
		
		// Clear the 'done' flag for this chain of transfers
		for(old_xfer = new_xfer; old_xfer != NULL; old_xfer = old_xfer->next){
			old_xfer->done = 0;
		}

		// Disable interrupts to modify the I2C transfer list
		__disable_irq();
		old_xfer = xfer;
		if(old_xfer != NULL){
			for(;old_xfer->next != NULL; old_xfer = old_xfer->next);
			old_xfer->next = new_xfer;
		} else {
			run_xfer(new_xfer);
		}
		__enable_irq();	
	}
		
protected:
	transfertype * xfer;
	
	const I2C_HWConfig &config;
	
	/*!
	 @brief Begin running this transfer, skipping all checks
	 @param new_xfer The transfer to start
	*/
	virtual void run_xfer(transfertype * __restrict new_xfer) = 0;
	
	static int check_evt(uint32_t event1, uint32_t event2){
		if((event1 & event2) == event2)
			return 1;
		return 0;
	}
	
	void next_xfer(){
		if(xfer->next){
			run_xfer(xfer->next);
		} else {
			xfer = nullptr;
		}
	}
};

/*!
 @brief Specialization for I2C master using 7-bit device addresses and 
 8-bit register addresses
 */
class I2CMaster_7bDev_8bRegaddrs : public I2CMaster< I2CMaster_7bDev_8bRegaddrs_Xfer > {
public:
	/*!
	 @param config The configuration to use for this device
	 */
	I2CMaster_7bDev_8bRegaddrs(I2C_HWConfig const &config) :
		I2CMaster< xfer_t >(config), state(I2C_ST_IDLE){}
	
	typedef I2CMaster_7bDev_8bRegaddrs_Xfer xfer_t;
	
	void write_byte(uint8_t devaddr, uint8_t addr, uint8_t value);
	
	uint8_t read_byte(uint8_t devaddr, uint8_t addr);
	
	void isr_event();
	
	void isr_error();
	
protected:
	void run_xfer(xfer_t * __restrict new_xfer){
		xfer = new_xfer;

		state = I2C_ST_MASTER_REQ;
		
		while(config.i2c->SR2 & I2C_SR2_BUSY);

		I2C_AcknowledgeConfig(config.i2c, ENABLE);
		I2C_GenerateSTART(config.i2c, ENABLE);	
	}
	
	//! State machine declaration for this I2C mode
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
	} state_t;
		
	state_t state;
	
	inline void isr_error_write();
	inline void isr_error_read();
	inline void isr_event_write();
	inline void isr_event_read();
};


#if USE_I2C1
//extern i2c_t i2c1;
#endif

#if USE_I2C2
//extern i2c_t i2c2;
#endif

#if USE_I2C3
//extern i2c_t i2c3;
#endif

#if 0
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

void i2c_write_byte(i2c_t *i2c, uint8_t devaddr, uint8_t addr, uint8_t value);

uint8_t i2c_read_byte(i2c_t *i2c, uint8_t devaddr, uint8_t addr);

/*!
 @brief Start or queue a transfer
 */
void i2c_transfer(i2c_t *i2c, i2c_transfer_t *xfer);
#endif

#endif

