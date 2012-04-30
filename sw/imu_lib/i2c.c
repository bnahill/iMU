#include "stm32f4xx_conf.h"
#include "i2c.h"

#if USE_I2C1
static const i2c_config_t i2c1_config = {
	GPIO_AF_I2C1,
	// SDA
	GPIOB,
	BIT(9),
	GPIO_PinSource9,
	// SCL
	GPIOB,
	BIT(8),
	GPIO_PinSource8,
	// IRQs
	I2C1_EV_IRQn,
	I2C1_ER_IRQn,
	// Clock
	RCC_APB1Periph_I2C1
};

i2c_t i2c1 = {
	I2C1,
	I2C_MODE_DISABLED,
	I2C_OP_READ,
	I2C_ST_IDLE,
	0,
	0,
	(uint8_t *)0,
	0,
	(uint8_t *)0,
	0,
	&i2c1_config
};
#endif

#if USE_I2C2
static const i2c_config_t i2c2_config = {
	GPIO_AF_I2C2,
	// SDA
	GPIOB,
	BIT(11),
	GPIO_PinSource9,
	// SCL
	GPIOB,
	BIT(10),
	GPIO_PinSource8,
	// IRQs
	I2C2_EV_IRQn,
	I2C2_ER_IRQn,
	// Clock
	RCC_APB1Periph_I2C2
};

i2c_t i2c2 = {
	I2C2,
	I2C_MODE_DISABLED,
	I2C_OP_READ,
	I2C_ST_IDLE,
	0,
	0,
	(uint8_t *)0,
	0,
	(uint8_t *)0,
	0,
	&i2c2_config
};
#endif

#if USE_I2C3
static const i2c_config_t i2c3_config = {
	GPIO_AF_I2C3,
	// SDA
	GPIOC,
	BIT(9),
	GPIO_PinSource9,
	// SCL
	GPIOC,
	BIT(8),
	GPIO_PinSource8,
	// IRQs
	I2C3_EV_IRQn,
	I2C3_ER_IRQn,
	// Clock
	RCC_APB1Periph_I2C3
};

i2c_t i2c3 = {
	I2C3,
	I2C_MODE_DISABLED,
	I2C_OP_READ,
	I2C_ST_IDLE,
	0,
	0,
	(uint8_t *)0,
	0,
	(uint8_t *)0,
	0,
	&i2c3_config
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
	
	// All I2C peripherals are on APB1
	RCC_APB1PeriphClockCmd(conf->clock, ENABLE);
	I2C_DeInit(i2c->i2c);

	////////////////////////////////////////////////////////////////////
	// GPIO Config
	////////////////////////////////////////////////////////////////////
	
	// Alternate functions
	GPIO_PinAFConfig(conf->sda_gpio, conf->sda_pinsrc, conf->af);
	GPIO_PinAFConfig(conf->scl_gpio, conf->scl_pinsrc, conf->af);

	// Mode configuration
	gpio_init_s.GPIO_Mode = GPIO_Mode_AF;
	gpio_init_s.GPIO_Speed = GPIO_Speed_100MHz;
	gpio_init_s.GPIO_OType = GPIO_OType_OD;
	
	gpio_init_s.GPIO_Pin = conf->sda_pin;
	GPIO_Init(conf->sda_gpio, &gpio_init_s);
	gpio_init_s.GPIO_Pin = conf->scl_pin;
	GPIO_Init(conf->scl_gpio, &gpio_init_s);

	////////////////////////////////////////////////////////////////////
	// I2C Config
	////////////////////////////////////////////////////////////////////

	i2c_init_s.I2C_Mode = I2C_Mode_I2C;
	i2c_init_s.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c_init_s.I2C_ClockSpeed = speed;
	i2c_init_s.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c_init_s.I2C_Ack = I2C_Ack_Enable;

	I2C_Init(i2c->i2c, &i2c_init_s);

	////////////////////////////////////////////////////////////////////
	// Interrupt Config
	////////////////////////////////////////////////////////////////////
	
	nvic_init_s.NVIC_IRQChannel = conf->irq_er;
	nvic_init_s.NVIC_IRQChannelSubPriority = 2;
	nvic_init_s.NVIC_IRQChannelPreemptionPriority = 2;
	nvic_init_s.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init_s);

	nvic_init_s.NVIC_IRQChannel = conf->irq_ev;
	nvic_init_s.NVIC_IRQChannelSubPriority = 2;
	nvic_init_s.NVIC_IRQChannelPreemptionPriority = 2;
	nvic_init_s.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init_s);

	I2C_ITConfig(i2c->i2c, I2C_IT_ERR, ENABLE);
	I2C_ITConfig(i2c->i2c, I2C_IT_BUF, ENABLE);
	I2C_ITConfig(i2c->i2c, I2C_IT_EVT, ENABLE);

	////////////////////////////////////////////////////////////////////
	// Finalize
	////////////////////////////////////////////////////////////////////

	I2C_Cmd(i2c->i2c, ENABLE);

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

void i2c_write(i2c_t *i2c, uint8_t devaddr, uint8_t addr, uint8_t *buffer, uint8_t count, int8_t *flag){
	i2c_spinlock(i2c);
	i2c->op = I2C_OP_WRITE;
	i2c->addr = addr;
	i2c->devaddr = devaddr;
	i2c->count = count;
	i2c->state = I2C_ST_MASTER_REQ;
	I2C_AcknowledgeConfig(i2c->i2c, ENABLE);
	I2C_GenerateSTART(i2c->i2c, ENABLE);
}

extern int inline i2c_check_evt(uint32_t event1, uint32_t event2){
	if((event1 & event2) == event2)
		return 1;
	return 0;
}

extern void inline i2c_isr_evt(i2c_t *restrict const i2c){
	uint32_t const event = I2C_GetLastEvent(i2c->i2c);
	switch(i2c->op){
	case I2C_OP_WRITE:
		switch(i2c->state){
		case I2C_ST_MASTER_REQ:
			if(i2c_check_evt(event, I2C_EVENT_MASTER_MODE_SELECT)){
				I2C_Send7bitAddress(i2c->i2c, i2c->devaddr, I2C_Direction_Transmitter);
				i2c->state = I2C_ST_ADDRESSED;
			}
			break;
		case I2C_ST_ADDRESSED:
			if(i2c_check_evt(event, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
				I2C_SendData(i2c->i2c, i2c->addr);
				i2c->state = I2C_ST_ADDR_TXED;
			}
			break;
		case I2C_ST_ADDR_TXED:
			if(I2C_GetFlagStatus(i2c->i2c, I2C_FLAG_BTF)){
				I2C_SendData(i2c->i2c, *(i2c->buffer));
				if(--i2c->count == 0){
					I2C_GenerateSTOP(i2c->i2c, ENABLE);
				}
			}
		}
		break;
	case I2C_OP_READ:
		break;
	}
}

extern void inline i2c_isr_err(i2c_t *restrict i2c){

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
