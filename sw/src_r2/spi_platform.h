#ifndef _SPI_C__
#error "spi_platform.h must only be included by spi.c!"
#endif

#if USE_SPI1
static const spi_config_t spi1_config = {
	.af = GPIO_AF_SPI1,
	.miso = {
		.gpio = GPIOB,
		.pin = BIT(4),
		.pinsrc = GPIO_PinSource4,
	},
	.mosi = {
		.gpio = GPIOB,
		.pin = BIT(5),
		.pinsrc = GPIO_PinSource5,
	},
	.sclk = {
		.gpio = GPIOA,
		.pin = BIT(5),
		.pinsrc = GPIO_PinSource5,
	},
	.dma_rx_stream = DMA2_Stream0,
	.dma_tx_stream = DMA2_Stream3,
	.dma = DMA2,
	.dma_channel = DMA_Channel_3,
	.dma_rx_tcif = DMA_IT_TCIF0,
	.dma_rx_tc_flag = DMA_FLAG_TCIF0,
	.dma_tx_tc_flag = DMA_FLAG_TCIF3,
	.dma_irq = DMA2_Stream0_IRQn,
	.dma_clock = RCC_AHB1Periph_DMA2,
	.clock_cmd = RCC_APB2PeriphClockCmd,
	.clock = RCC_APB2Periph_SPI1
};

spi_t spi1 = {
	.spi = SPI1,
	.xfer = NULL,
	.is_init = 0,
	.config = &spi1_config
};
#define SPI1_DMA_ISR DMA2_Stream0_IRQHandler
#endif

#if USE_SPI1
void SPI1_DMA_ISR(void){spi_dma_isr(&spi1);}
#endif

#if USE_SPI2
void SPI2_DMA_ISR(void){spi_dma_isr(&spi2);}
#endif
