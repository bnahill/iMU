#include "stm32f4xx_conf.h"
#include "i2c.h"

#if USE_I2C1
static const i2c_config_t i2c1_config = {
	.af = GPIO_AF_I2C1,
	// SDA
	.sda_gpio = GPIOB,
	.sda_pin = BIT(9),
	.sda_pinsrc = GPIO_PinSource9,
	// SCL
	.scl_gpio = GPIOB,
	.scl_pin = BIT(8),
	.scl_pinsrc = GPIO_PinSource8,
	// IRQs
	.irq_ev = I2C1_EV_IRQn,
	.irq_er = I2C1_ER_IRQn,
	// Clock
	.clock = RCC_APB1Periph_I2C1
};

i2c_t i2c1 = {
	.i2c = I2C1,
	.mode = I2C_MODE_DISABLED,
	.state = I2C_ST_IDLE,
	.config = &i2c1_config
};
#endif

#if USE_I2C2
static const i2c_config_t i2c2_config = {
	.af = GPIO_AF_I2C2,
	// SDA
	.sda_gpio = GPIOB,
	.sda_pin = BIT(11),
	.sda_pinsrc = GPIO_PinSource11,
	// SCL
	.scl_gpio = GPIOB,
	.scl_pin = BIT(10),
	.scl_pinsrc = GPIO_PinSource10,
	// IRQs
	.irq_ev = I2C2_EV_IRQn,
	.irq_er = I2C2_ER_IRQn,
	// Clock
	.clock = RCC_APB1Periph_I2C2
};

i2c_t i2c2 = {
	.i2c = I2C2,
	.mode = I2C_MODE_DISABLED,
	.state = I2C_ST_IDLE,
	.config = &i2c2_config
};
#endif

#if USE_I2C3
static const i2c_config_t i2c3_config = {
	.af = GPIO_AF_I2C3,
	// SDA
	.sda_gpio = GPIOC,
	.sda_pin = BIT(9),
	.sda_pinsrc = GPIO_PinSource9,
	// SCL
	.scl_gpio = GPIOC,
	.scl_pin = BIT(8),
	.scl_pinsrc = GPIO_PinSource8,
	// IRQs
	.irq_ev = I2C3_EV_IRQn,
	.irq_er = I2C3_ER_IRQn,
	// Clock
	.clock = RCC_APB1Periph_I2C3
};

i2c_t i2c3 = {
	.i2c = I2C3
	.mode = I2C_MODE_DISABLED,
	.state = I2C_ST_IDLE,
	.config = &i2c3_config
};
#endif


void inline i2c_spinlock(i2c_t *i2c);
int inline i2c_trylock(i2c_t *i2c);
void i2c_unlock(i2c_t *i2c);


int i2c_init(i2c_t *i2c, i2c_mode_t mode, uint32_t speed){
	i2c_config_t const * const conf = i2c->config;
	GPIO_InitTypeDef gpio_init_s;
	I2C_InitTypeDef i2c_init_s;
	NVIC_InitTypeDef nvic_init_s;

	__disable_irq();
	if(i2c->mode != I2C_MODE_DISABLED){
		__enable_irq();
		return 1;
	}

	GPIO_StructInit(&gpio_init_s);
	I2C_StructInit(&i2c_init_s);

	// All I2C peripherals are on APB1
	RCC_APB1PeriphClockCmd(conf->clock, ENABLE);
	
	// Reset
	RCC_APB1PeriphResetCmd(conf->clock, ENABLE);
	RCC_APB1PeriphResetCmd(conf->clock, DISABLE);

	////////////////////////////////////////////////////////////////////
	// GPIO Config
	////////////////////////////////////////////////////////////////////

	// Set alternate functions to use
	GPIO_PinAFConfig(conf->sda_gpio, conf->sda_pinsrc, conf->af);
	GPIO_PinAFConfig(conf->scl_gpio, conf->scl_pinsrc, conf->af);
/*
	// Mode configuration
	gpio_init_s.GPIO_Mode = GPIO_Mode_IN;
	gpio_init_s.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init_s.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_init_s.GPIO_OType = GPIO_OType_OD;

	gpio_init_s.GPIO_Pin = conf->sda_pin;
	GPIO_Init(conf->sda_gpio, &gpio_init_s);
	gpio_init_s.GPIO_Pin = conf->scl_pin;
	GPIO_Init(conf->scl_gpio, &gpio_init_s);
*/

	// Mode configuration
	gpio_init_s.GPIO_Mode = GPIO_Mode_AF;
	gpio_init_s.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init_s.GPIO_PuPd = GPIO_PuPd_UP;
	gpio_init_s.GPIO_OType = GPIO_OType_OD;

	// For some reason the order here matters and it really shouldn't
	gpio_init_s.GPIO_Pin = conf->scl_pin;
	GPIO_Init(conf->scl_gpio, &gpio_init_s);
	gpio_init_s.GPIO_Pin = conf->sda_pin;
	GPIO_Init(conf->sda_gpio, &gpio_init_s);


/*
	while(1){
		conf->scl_gpio->BSRRL = conf->scl_pin;
		conf->sda_gpio->BSRRH = conf->sda_pin;
		conf->sda_gpio->BSRRL = conf->sda_pin;
		conf->scl_gpio->BSRRH = conf->scl_pin;
		GPIOB->BSRRL = BIT(8);
		GPIOB->BSRRL = BIT(9);
		GPIOB->BSRRH = BIT(8);
		GPIOB->BSRRH = BIT(9);
	}
*/	

	////////////////////////////////////////////////////////////////////
	// I2C Config
	////////////////////////////////////////////////////////////////////

	i2c_init_s.I2C_Mode = I2C_Mode_I2C;
	i2c_init_s.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c_init_s.I2C_ClockSpeed = 100000;
	i2c_init_s.I2C_OwnAddress1 = 0xA0;
	i2c_init_s.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c_init_s.I2C_Ack = I2C_Ack_Enable;

	////////////////////////////////////////////////////////////////////
	// Interrupt Config
	////////////////////////////////////////////////////////////////////

	nvic_init_s.NVIC_IRQChannel = conf->irq_er;
	nvic_init_s.NVIC_IRQChannelSubPriority = 2;
	nvic_init_s.NVIC_IRQChannelPreemptionPriority = 2;
	nvic_init_s.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init_s);

	nvic_init_s.NVIC_IRQChannel = conf->irq_ev;
	nvic_init_s.NVIC_IRQChannelSubPriority = 3;
	nvic_init_s.NVIC_IRQChannelPreemptionPriority = 3;
	nvic_init_s.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init_s);

	I2C_ITConfig(i2c->i2c, I2C_IT_ERR, ENABLE);
	I2C_ITConfig(i2c->i2c, I2C_IT_BUF, ENABLE);
	I2C_ITConfig(i2c->i2c, I2C_IT_EVT, ENABLE);

	////////////////////////////////////////////////////////////////////
	// Finalize
	////////////////////////////////////////////////////////////////////

	I2C_Cmd(i2c->i2c, ENABLE);

	I2C_Init(i2c->i2c, &i2c_init_s);

	i2c->mode = mode;

	__enable_irq();
	return 1;
}

void i2c_spinlock(i2c_t *i2c){
	uint8_t i;
	while(1){
		if(i2c_trylock(i2c))
			break;
		for(i = 0; i != 0xFF; i++);
	}
}

int i2c_trylock(i2c_t *i2c){
	__disable_irq();
	if(i2c->lock){
		__enable_irq();
		return 0;
	}
	i2c->lock = 1;
	__enable_irq();
	return 1;
}

void i2c_unlock(i2c_t *i2c){
	i2c->lock = 0;
}

/*!
 @brief Begin running this transfer, skipping all checks
 @param i2c The I2C device
 @param xfer The transfer to start

 This is intended to be used internally to start each transfer
 */
extern void inline i2c_run_xfer(i2c_t *i2c, i2c_transfer_t *xfer){
	i2c->xfer = xfer;
	
	i2c->state = I2C_ST_MASTER_REQ;
	while(i2c->i2c->SR2 & I2C_SR2_BUSY);

	I2C_AcknowledgeConfig(i2c->i2c, ENABLE);
	I2C_GenerateSTART(i2c->i2c, ENABLE);
}

void i2c_transfer(i2c_t *i2c, i2c_transfer_t *xfer){
	i2c_transfer_t *old_xfer;
	// Clear the 'done' flag for this chain of transfers
	for(old_xfer = xfer; old_xfer != NULL; old_xfer = old_xfer->next){
		old_xfer->done = 0;
	}

	// Disable interrupts to modify the I2C transfer list
	__disable_irq();
	old_xfer = i2c->xfer;
	if(old_xfer != NULL){
		for(;old_xfer->next != NULL; old_xfer = old_xfer->next);
		old_xfer->next = xfer;
	} else {
		i2c_run_xfer(i2c, xfer);	
	}
	__enable_irq();
}

void i2c_write_byte(i2c_t *i2c, uint8_t devaddr, uint8_t addr, uint8_t value){
	i2c_transfer_t xfer = {
		.op = I2C_OP_WRITE,
		.devaddr = devaddr,
		.addr = addr,
		.buffer = &value,
		.count = 1,
		.next = NULL
	};
	i2c_transfer(i2c, &xfer);
	while(!xfer.done);
}

uint8_t i2c_read_byte(i2c_t *i2c, uint8_t devaddr, uint8_t addr){
	uint8_t result;
	i2c_transfer_t xfer = {
		.op = I2C_OP_READ,
		.devaddr = devaddr,
		.addr = addr,
		.buffer = &result,
		.count = 1,
		.next = NULL
	};
	i2c_transfer(i2c, &xfer);
	while(!xfer.done);
	return result;
}

/*
void i2c_write(i2c_t *i2c, uint8_t devaddr, uint8_t addr, uint8_t *buffer, uint8_t count){
	i2c_spinlock(i2c);
	i2c->op = I2C_OP_WRITE;
	i2c->addr = addr;
	i2c->devaddr = devaddr;
	i2c->count = count;
	i2c->buffer = buffer;
	i2c->state = I2C_ST_MASTER_REQ;
	i2c->done = 0;

	while(i2c->i2c->SR2 & I2C_SR2_BUSY);

	I2C_AcknowledgeConfig(i2c->i2c, ENABLE);
	I2C_GenerateSTART(i2c->i2c, ENABLE);
}

void i2c_read(i2c_t *i2c, uint8_t devaddr, uint8_t addr, uint8_t *buffer, uint8_t count){
	i2c_spinlock(i2c);
	i2c->op = I2C_OP_READ;
	i2c->addr = addr;
	i2c->devaddr = devaddr;
	i2c->count = count;
	i2c->buffer = buffer;
	i2c->state = I2C_ST_MASTER_REQ;
	i2c->done = 0;

	while(i2c->i2c->SR2 & I2C_SR2_BUSY);

	I2C_AcknowledgeConfig(i2c->i2c, ENABLE);
	I2C_GenerateSTART(i2c->i2c, ENABLE);
}
*/

extern int inline i2c_check_evt(uint32_t event1, uint32_t event2){
	if((event1 & event2) == event2)
		return 1;
	return 0;
}

extern void inline i2c_next_xfer(i2c_t *RESTRICT const i2c){
	if(i2c->xfer->next){
		i2c_run_xfer(i2c, i2c->xfer->next);
	} else {
		i2c->xfer = NULL;
	}
}
extern void inline i2c_read_isr_evt(i2c_t *RESTRICT const i2c){
	uint32_t const event = I2C_GetLastEvent(i2c->i2c);
	switch(i2c->state){
	case I2C_ST_MASTER_REQ:
		if(event == I2C_EVENT_MASTER_MODE_SELECT){
			I2C_AcknowledgeConfig(i2c->i2c, ENABLE);
			I2C_Send7bitAddress(i2c->i2c, i2c->xfer->devaddr, I2C_Direction_Transmitter);
			i2c->state = I2C_ST_ADDRESSED;
		} else {
			while(1);
		}
		break;
	case I2C_ST_ADDRESSED:
		if(event == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED){
			(void)i2c->i2c->SR1;
			I2C_SendData(i2c->i2c, i2c->xfer->addr);
			i2c->state = I2C_ST_ADDR_TXED;
		}
		break;
	case I2C_ST_ADDR_TXED:
		if(I2C_GetFlagStatus(i2c->i2c, I2C_FLAG_BTF) == SET){
			I2C_GenerateSTART(i2c->i2c, ENABLE);
			i2c->state = I2C_ST_REPEAT_START;
		}
		break;
	case I2C_ST_REPEAT_START:
		if(event == I2C_EVENT_MASTER_MODE_SELECT){
			(void)i2c->i2c->SR1;
			I2C_Send7bitAddress(i2c->i2c, i2c->xfer->devaddr, I2C_Direction_Receiver);
			I2C_ITConfig(i2c->i2c, I2C_IT_BUF , ENABLE);
			i2c->state = I2C_ST_REPEAT_ADDR;
		}
		break;
	case I2C_ST_REPEAT_ADDR:
		if(event == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED){
			if(i2c->xfer->count  == 1){
				// NACK the next byte
				I2C_AcknowledgeConfig(i2c->i2c, DISABLE);
			}
			i2c->state = I2C_ST_READING;
		}
		break;
	case I2C_ST_READING:
		if(event == I2C_EVENT_MASTER_BYTE_RECEIVED){
			*(i2c->xfer->buffer++) = I2C_ReceiveData(i2c->i2c);
			if(--i2c->xfer->count == 1){
				// NACK the next byte
				I2C_AcknowledgeConfig(i2c->i2c, DISABLE);
			} else if (i2c->xfer->count == 0){
				i2c->state = I2C_ST_IDLE;
				I2C_GenerateSTOP(i2c->i2c, ENABLE);
				i2c->xfer->done = 1;
				i2c_next_xfer(i2c);
			}
		}
		break;
	}

}

extern void inline i2c_write_isr_evt(i2c_t *RESTRICT const i2c){
	uint32_t const event = I2C_GetLastEvent(i2c->i2c);
	switch(i2c->state){
	case I2C_ST_MASTER_REQ:
		if(i2c_check_evt(event, I2C_EVENT_MASTER_MODE_SELECT)){
			I2C_Send7bitAddress(i2c->i2c, i2c->xfer->devaddr, I2C_Direction_Transmitter);
			i2c->state = I2C_ST_ADDRESSED;
		}
		break;
	case I2C_ST_ADDRESSED:
		if(i2c_check_evt(event, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
			I2C_SendData(i2c->i2c, i2c->xfer->addr);
			i2c->state = I2C_ST_WRITING;
		}
		break;
	case I2C_ST_WRITING:
		if(I2C_GetFlagStatus(i2c->i2c, I2C_FLAG_BTF)){
			I2C_SendData(i2c->i2c, *(i2c->xfer->buffer++));
			if(--i2c->xfer->count == 0){
				I2C_GenerateSTOP(i2c->i2c, ENABLE);
				i2c->state = I2C_ST_CLOSING_WRITE;
			}
		}
		break;
	case I2C_ST_CLOSING_WRITE:
		i2c->xfer->done = 1;
		i2c->state = I2C_ST_IDLE;
		i2c_next_xfer(i2c);
		break;	
	}
}

extern void inline i2c_read_isr_err(i2c_t *RESTRICT const i2c){
	while(1);
}

extern void inline i2c_write_isr_err(i2c_t *RESTRICT const i2c){
	uint32_t const event = I2C_GetLastEvent(i2c->i2c);
	switch(i2c->state){
	case I2C_ST_IDLE:
		if((i2c->i2c->SR1 & 0xFF00) != 0){
			i2c->i2c->SR1 &= 0xFF00;
		} else if(event){
			while(1);
		}
		break;
	case I2C_ST_ADDRESSED:
		if(event & I2C_SR1_AF){
			i2c->xfer->done = I2C_XFER_ERR_NOSLAVE;
			i2c->i2c->SR1 = 0;
			i2c_next_xfer(i2c);
		}
		break;
	default:
		if(event)
			while(1);
		break;
	}
}
	
extern void inline i2c_isr_evt(i2c_t *RESTRICT const i2c){
	switch(i2c->xfer->op){
	case I2C_OP_WRITE:
		i2c_write_isr_evt(i2c);
		break;
	case I2C_OP_READ:
		i2c_read_isr_evt(i2c);
		break;
	default:
		while(1);
		break;
	}
}

extern void inline i2c_isr_err(i2c_t *RESTRICT i2c){
	switch(i2c->xfer->op){
	case I2C_OP_WRITE:
		i2c_write_isr_err(i2c);
		break;
	case I2C_OP_READ:
		i2c_read_isr_err(i2c);
		break;
	default:
		while(1);
		break;
	}

}

#if USE_I2C1
void I2C1_ER_IRQHandler(void){i2c_isr_err(&i2c1);}
void I2C1_EV_IRQHandler(void){i2c_isr_evt(&i2c1);}
#endif
#if USE_I2C2
void I2C2_ER_IRQHandler(void){i2c_isr_err(&i2c2);}
void I2C2_EV_IRQHandler(void){i2c_isr_evt(&i2c2);}
#endif
#if USE_I2C3
void I2C3_ER_IRQHandler(void){i2c_isr_err(&i2c3);}
void I2C3_EV_IRQHandler(void){i2c_isr_evt(&i2c3);}
#endif
