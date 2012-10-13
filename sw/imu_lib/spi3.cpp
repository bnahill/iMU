extern "C" {
	#include "stm32f4xx.h"
}
#include "spi.h"

#define __SPI3_CPP_

bool SPI_3Wire::init(){
	SPI_InitTypeDef  spi_init_s;
	GPIO_InitTypeDef gpio_init_s;
	NVIC_InitTypeDef nvic_init_s;
	DMA_InitTypeDef  dma_init_s;

	if(xfer != NULL)
		return false;
	
	GPIO_StructInit(&gpio_init_s);
	SPI_StructInit(&spi_init_s);
	DMA_StructInit(&dma_init_s);
	
	__disable_irq();
	if(is_init){
		__enable_irq();
		return true;
	}

	// Enable clock for DMA and SPI
	RCC_AHB1PeriphClockCmd(config.dma_clock, ENABLE);
	config.clock_cmd(config.clock, ENABLE);

	// Configure GPIOs
	gpio_init_s.GPIO_Speed = GPIO_Speed_100MHz;
	gpio_init_s.GPIO_Mode = GPIO_Mode_AF;
	gpio_init_s.GPIO_OType = GPIO_OType_PP;

	// SCLK
	GPIO_PinAFConfig(config.sclk.gpio, config.sclk.pinsrc, config.af);
	gpio_init_s.GPIO_Pin = config.sclk.pin;
	GPIO_Init(config.sclk.gpio, &gpio_init_s);

	// MOSI
	GPIO_PinAFConfig(config.mosi.gpio, config.mosi.pinsrc, config.af);
	gpio_init_s.GPIO_Pin = config.mosi.pin;
	GPIO_Init(config.mosi.gpio, &gpio_init_s);

	DMA_DeInit(config.dma_rx_stream);
	DMA_DeInit(config.dma_tx_stream);

	// Configure DMA streams
	dma_init_s.DMA_Channel = config.dma_channel;
	dma_init_s.DMA_PeripheralBaseAddr = (uint32_t) &spi->DR;
	dma_init_s.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_init_s.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_init_s.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma_init_s.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma_init_s.DMA_Mode = DMA_Mode_Normal;
	dma_init_s.DMA_Priority = DMA_Priority_VeryHigh;
	dma_init_s.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma_init_s.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma_init_s.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma_init_s.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	dma_init_s.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_Init(config.dma_rx_stream, &dma_init_s);
	dma_init_s.DMA_Priority = DMA_Priority_High;
	dma_init_s.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_Init(config.dma_tx_stream, &dma_init_s);

	// Configure SPI
	SPI_I2S_DeInit(spi);
	spi_init_s.SPI_Mode = SPI_Mode_Master;
	spi_init_s.SPI_Direction = SPI_Direction_1Line_Tx;
	spi_init_s.SPI_DataSize = SPI_DataSize_8b;
	spi_init_s.SPI_CPOL = SPI_CPOL_High;
	spi_init_s.SPI_CPHA = SPI_CPHA_2Edge;
	spi_init_s.SPI_NSS = SPI_NSS_Soft;
	spi_init_s.SPI_BaudRatePrescaler = config.prescaler;
	spi_init_s.SPI_FirstBit = SPI_FirstBit_MSB;
	spi_init_s.SPI_CRCPolynomial = 7;

	// Enable SPI
	SPI_Init(spi, &spi_init_s);
	SPI_Cmd(spi, ENABLE);

	SPI_DMACmd(spi, SPI_DMAReq_Rx, ENABLE);
	SPI_DMACmd(spi, SPI_DMAReq_Tx, ENABLE);

	// Configure DMA interrupts
	nvic_init_s.NVIC_IRQChannel = config.dma_rx_irq;
	nvic_init_s.NVIC_IRQChannelPreemptionPriority = 0;
	nvic_init_s.NVIC_IRQChannelSubPriority = 2;
	nvic_init_s.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init_s);

	// Also need an interrupt for TX complete
	nvic_init_s.NVIC_IRQChannelPreemptionPriority = 3;
	nvic_init_s.NVIC_IRQChannel = config.irq;
	NVIC_Init(&nvic_init_s);
	
	SPI_I2S_ITConfig(spi, SPI_I2S_IT_RXNE, DISABLE); 
	DMA_ITConfig(config.dma_tx_stream, DMA_IT_TC, DISABLE);
	

	is_init = true;
	__enable_irq();
	return true;
}

void SPI_3Wire::run_xfer(spi_transfer_t *_xfer){
	xfer = _xfer;

	// Set nSS
	if(xfer->nss){
		xfer->nss->gpio->BSRRH = xfer->nss->pin;
	}

	// Configure DMA transfers
	config.dma_rx_stream->M0AR = (uint32_t)xfer->read_buff;
	config.dma_rx_stream->NDTR = xfer->write_count;
	config.dma_tx_stream->M0AR = (uint32_t)xfer->write_buff;
	config.dma_tx_stream->NDTR = xfer->write_count;
	
	// Enable output
	spi->CR1 &= ~SPI_CR1_BIDIMODE;
	spi->CR1 |= SPI_CR1_BIDIOE;
	
	spi->CR1 |= SPI_CR1_SPE;
	
	DMA_ITConfig(config.dma_rx_stream, DMA_IT_TC, ENABLE);
	config.dma_rx_stream->CR |= DMA_SxCR_EN;
	
	config.dma_tx_stream->CR |= DMA_SxCR_EN;
}

#include "spi_platform.h"
