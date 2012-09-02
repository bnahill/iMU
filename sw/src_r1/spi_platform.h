#ifndef __SPI_C_
#error "spi_platform.h must only be included by spi.c!"
#endif

#include "spi.h"
#include "spi_private.h"

static const spi_config_t spi1_config = {
	.mode = SPI_MODE_3WIRE,
	.af = GPIO_AF_SPI1,
	.miso = {
		.gpio = GPIOA,
		.pin = BIT(6),
		.pinsrc = GPIO_PinSource6,
	},
	.mosi = {
		.gpio = GPIOA,
		.pin = BIT(7),
		.pinsrc = GPIO_PinSource7,
	},
	.sclk = {
		.gpio = GPIOA,
		.pin = BIT(5),
		.pinsrc = GPIO_PinSource5,
	},
	.prescaler = SPI_BaudRatePrescaler_64,
	.dma_rx_stream = DMA2_Stream0,
	.dma_tx_stream = DMA2_Stream3,
	.dma = DMA2,
	.dma_channel = DMA_Channel_3,
	.dma_rx_tcif = DMA_IT_TCIF0,
	.dma_rx_tc_flag = DMA_FLAG_TCIF0,
	.dma_tx_tcif = DMA_IT_TCIF3,
	.dma_tx_tc_flag = DMA_FLAG_TCIF3,
	.dma_rx_irq = DMA2_Stream0_IRQn,
	.dma_tx_irq = DMA2_Stream3_IRQn,
	.dma_clock = RCC_AHB1Periph_DMA2,
	.irq = SPI1_IRQn,
	.clock_cmd = RCC_APB2PeriphClockCmd,
	.clock = RCC_APB2Periph_SPI1
};

spi_t spi1 = {
	.spi = SPI1,
	.xfer = NULL,
	.is_init = 0,
	.config = &spi1_config
};

#define SPI1_RX_DMA_ISR  DMA2_Stream0_IRQHandler
#define SPI1_TX_DMA_ISR  DMA2_Stream3_IRQHandler
#define SPI1_RX_ISR      SPI1_IRQHandler

void SPI1_RX_ISR(void){spi_rx_isr(&spi1);}
void SPI1_RX_DMA_ISR(void){spi_rx_dma_isr(&spi1);}
void SPI1_TX_DMA_ISR(void){spi_tx_dma_isr(&spi1);}
