#include "stm32f4xx_conf.h"

#include "tick.h"
#include "l3gd20.h"
#include "lsm303.h"
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
	printf("apple");
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
/*	
	if((SD_Init() == SD_OK) && SD_Detect()){
		sderr = SD_GetCardInfo(&ci);
		if(sderr == SD_OK){
			while(1);
		}
	}
*/	

	lsm303_init();
	l3gd20_init();

	
	while (1){
		tick_wait(1);
		l3gd20_read_sync();
		lsm303_read();
	}
}

