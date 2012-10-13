#ifndef __SPI_PLATFORM_C_
#define __SPI_PLATFORM_C_

extern SPI_4Wire spi1;

#endif

#ifdef __SPI4_CPP_

///////////////////////////////////////////
// Declare all 4-wire SPI peripherals here
///////////////////////////////////////////

const SPI_4WireConfig SPI1_Config = {
	GPIO_AF_SPI1,
	{GPIOB, BIT(4), GPIO_PinSource4},
	{GPIOB, BIT(5), GPIO_PinSource5},
	{GPIOA, BIT(5), GPIO_PinSource5},
	1,
	DMA2_Stream0,
	DMA2_Stream3,
	DMA2,
	DMA_Channel_3,
	DMA_IT_TCIF0,
	DMA_FLAG_TCIF0,
	DMA_FLAG_TCIF3,
	DMA2_Stream0_IRQn,
	RCC_AHB1Periph_DMA2,
	SPI1_IRQn,
	RCC_APB2PeriphClockCmd,
	RCC_APB2Periph_SPI1
};


SPI_4Wire spi1(SPI1_Config, SPI1);

extern "C" {void DMA2_Stream0_IRQHandler();}
void DMA2_Stream0_IRQHandler(){
	spi1.isr_dma_rx();
}


#endif

#ifdef __SPI3_CPP_

///////////////////////////////////////////
// Declare all 3-wire SPI peripherals here
///////////////////////////////////////////



#endif
