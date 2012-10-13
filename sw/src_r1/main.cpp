#include "stm32f4xx_conf.h"

#include "tick.h"
#include "l3gd20_public.h"
#include "adxl345_public.h"
#include "sdio.h"
#include "stdlib.h"

//! @defgroup util Utilities

//! @name LED macros
//! @{
#define LED_GPIO GPIOD
#define LED_PIN(x) (GPIO_Pin_12 << x)
#define LED_PIN_ALL (LED_PIN(0) | LED_PIN(1) | LED_PIN(2))

#define LED_SET(x)    (LED_GPIO->ODR |= LED_PIN(x))
#define LED_SET_ALL() (LED_GPIO->ODR |= LED_PIN_ALL)
#define LED_CLR(x)    (LED_GPIO->ODR &= ~LED_PIN(x))
#define LED_CLR_ALL() (LED_GPIO->ODR &= ~LED_PIN_ALL)
//! @}

int main(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	SD_Error sderr;
	SD_CardInfo ci;

	// All GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
	                       RCC_AHB1Periph_GPIOB |
	                       RCC_AHB1Periph_GPIOC |
	                       RCC_AHB1Periph_GPIOD |
	                       RCC_AHB1Periph_GPIOE, ENABLE);

	// Configure LEDs in output pushpull mode
	GPIO_InitStructure.GPIO_Pin = LED_PIN_ALL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);
	
	// Configure SysTick for 400ms period	
	if(!tick_start(400.0)){
		while(1);
	}
	
	adxl345_init(&acc1);
	l3gd20_init();
	
	while (1){
		tick_wait(1);
		l3gd20_read_sync();
	}
}

