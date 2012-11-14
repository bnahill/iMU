#include "stm32f4xx_conf.h"
#include "i2c.h"

#define __I2C_CPP_

#include "i2c_platform.h"


#if USE_I2C1 && 0
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

#if USE_I2C2 && 0
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

void I2CMaster_7bDev_8bRegaddrs::write_byte(uint8_t devaddr, uint8_t addr, uint8_t value){
	xfer_t new_xfer(I2C_OP_WRITE, devaddr, addr, &value, 1);
	transfer(&new_xfer);
	while(!new_xfer.done);
}


uint8_t I2CMaster_7bDev_8bRegaddrs::read_byte(uint8_t devaddr, uint8_t addr){
	uint8_t result;
	xfer_t new_xfer(I2C_OP_READ, devaddr, addr, &result, 1);
	
	transfer(&new_xfer);
	
	while(!new_xfer.done);
	return result;
}


void I2CMaster_7bDev_8bRegaddrs::isr_event(){
	switch(xfer->op){
	case I2C_OP_WRITE:
		isr_event_write();
		break;
	case I2C_OP_READ:
		isr_event_read();
		break;
	default:
		while(1);
		break;
	}
}
	
void I2CMaster_7bDev_8bRegaddrs::isr_error(){
	switch(xfer->op){
	case I2C_OP_WRITE:
		isr_error_write();
		break;
	case I2C_OP_READ:
		isr_error_read();
		break;
	default:
		while(1);
		break;
	}
}

void I2CMaster_7bDev_8bRegaddrs::isr_error_read(){
	while(1);
}

void I2CMaster_7bDev_8bRegaddrs::isr_error_write(){
	uint32_t const event = I2C_GetLastEvent(config.i2c);
	switch(state){
	case I2C_ST_IDLE:
		if((config.i2c->SR1 & 0xFF00) != 0){
			config.i2c->SR1 &= 0xFF00;
		} else if(event){
			while(1);
		}
		break;
	case I2C_ST_ADDRESSED:
		if(event & I2C_SR1_AF){
			xfer->done = I2C_XFER_ERR_NOSLAVE;
			config.i2c->SR1 = 0;
			next_xfer();
		}
		break;
	default:
		if(event)
			while(1);
		break;
	}
}

void I2CMaster_7bDev_8bRegaddrs::isr_event_read(){
	uint32_t const event = I2C_GetLastEvent(config.i2c);
	switch(state){
	case I2C_ST_MASTER_REQ:
		if(event == I2C_EVENT_MASTER_MODE_SELECT){
			I2C_AcknowledgeConfig(config.i2c, ENABLE);
			I2C_Send7bitAddress(config.i2c, xfer->devaddr, I2C_Direction_Transmitter);
			state = I2C_ST_ADDRESSED;
		} else {
			while(1);
		}
		break;
	case I2C_ST_ADDRESSED:
		if(event == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED){
			(void)config.i2c->SR1;
			I2C_SendData(config.i2c, xfer->addr);
			state = I2C_ST_ADDR_TXED;
		}
		break;
	case I2C_ST_ADDR_TXED:
		if(I2C_GetFlagStatus(config.i2c, I2C_FLAG_BTF) == SET){
			I2C_GenerateSTART(config.i2c, ENABLE);
			state = I2C_ST_REPEAT_START;
		}
		break;
	case I2C_ST_REPEAT_START:
		if(event == I2C_EVENT_MASTER_MODE_SELECT){
			(void)config.i2c->SR1;
			I2C_Send7bitAddress(config.i2c, xfer->devaddr, I2C_Direction_Receiver);
			I2C_ITConfig(config.i2c, I2C_IT_BUF , ENABLE);
			state = I2C_ST_REPEAT_ADDR;
		}
		break;
	case I2C_ST_REPEAT_ADDR:
		if(event == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED){
			if(xfer->count  == 1){
				// NACK the next byte
				I2C_AcknowledgeConfig(config.i2c, DISABLE);
			}
			state = I2C_ST_READING;
		}
		break;
	case I2C_ST_READING:
		if(event == I2C_EVENT_MASTER_BYTE_RECEIVED){
			*(xfer->buffer++) = I2C_ReceiveData(config.i2c);
			if(--xfer->count == 1){
				// NACK the next byte
				I2C_AcknowledgeConfig(config.i2c, DISABLE);
			} else if (xfer->count == 0){
				state = I2C_ST_IDLE;
				I2C_GenerateSTOP(config.i2c, ENABLE);
				xfer->done = 1;
				this->next_xfer();
			}
		}
		break;
	}
}

void I2CMaster_7bDev_8bRegaddrs::isr_event_write(){
	uint32_t const event = I2C_GetLastEvent(config.i2c);
	switch(state){
	case I2C_ST_MASTER_REQ:
		if(check_evt(event, I2C_EVENT_MASTER_MODE_SELECT)){
			I2C_Send7bitAddress(config.i2c, xfer->devaddr, I2C_Direction_Transmitter);
			state = I2C_ST_ADDRESSED;
		}
		break;
	case I2C_ST_ADDRESSED:
		if(check_evt(event, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
			I2C_SendData(config.i2c, xfer->addr);
			state = I2C_ST_WRITING;
		}
		break;
	case I2C_ST_WRITING:
		if(I2C_GetFlagStatus(config.i2c, I2C_FLAG_BTF)){
			I2C_SendData(config.i2c, *(xfer->buffer++));
			if(--xfer->count == 0){
				I2C_GenerateSTOP(config.i2c, ENABLE);
				state = I2C_ST_CLOSING_WRITE;
			}
		}
		break;
	case I2C_ST_CLOSING_WRITE:
		xfer->done = 1;
		state = I2C_ST_IDLE;
		next_xfer();
		break;	
	}
}
