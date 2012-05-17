#include "stm32f4xx_conf.h"
#include "spi.h"

#if USE_SPI1
const spi_config_t spi1_config = {
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
	.dma_irq = DMA2_Stream0_IRQn,
	.dma_rx_tcif = DMA_IT_TCIF0,
	.dma_rx_tc_flag = DMA_FLAG_TCIF0,
	.dma_rx_tc_flag = DMA_FLAG_TCIF3,
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

void spi_init_slave(const gpio_pin_t *pin){
	GPIO_InitTypeDef gpio_init_s;
	
	GPIO_StructInit(&gpio_init_s);
	
	gpio_init_s.GPIO_Speed = GPIO_Speed_100MHz;
	gpio_init_s.GPIO_Mode = GPIO_Mode_OUT;
	gpio_init_s.GPIO_OType = GPIO_OType_PP;
	gpio_init_s.GPIO_Pin = pin->pin;

	pin->gpio->BSRRL = pin->pin;
	
	GPIO_Init(pin->gpio, &gpio_init_s);
}

void spi_init(spi_t *spi){
	SPI_InitTypeDef  spi_init_s;
	GPIO_InitTypeDef gpio_init_s;
	NVIC_InitTypeDef nvic_init_s;
	DMA_InitTypeDef  dma_init_s;

	const spi_config_t *conf = spi->config;
	if(spi->xfer != NULL)
		return;
	
	GPIO_StructInit(&gpio_init_s);
	SPI_StructInit(&spi_init_s);
	DMA_StructInit(&dma_init_s);
	
	__disable_irq();
	if(spi->is_init){
		__enable_irq();
		return;
	}

	// Enable clock for DMA and SPI
	RCC_AHB1PeriphClockCmd(conf->dma_clock, ENABLE);
	conf->clock_cmd(conf->clock, ENABLE);

	// Configure GPIOs
	gpio_init_s.GPIO_Speed = GPIO_Speed_100MHz;
	gpio_init_s.GPIO_Mode = GPIO_Mode_AF;
	gpio_init_s.GPIO_OType = GPIO_OType_PP;

	GPIO_PinAFConfig(conf->miso.gpio, conf->miso.pinsrc, conf->af);
	gpio_init_s.GPIO_Pin = conf->miso.pin;
	GPIO_Init(conf->miso.gpio, &gpio_init_s);
	
	GPIO_PinAFConfig(conf->mosi.gpio, conf->mosi.pinsrc, conf->af);
	gpio_init_s.GPIO_Pin = conf->mosi.pin;
	GPIO_Init(conf->mosi.gpio, &gpio_init_s);
	
	GPIO_PinAFConfig(conf->sclk.gpio, conf->sclk.pinsrc, conf->af);
	gpio_init_s.GPIO_Pin = conf->sclk.pin;
	GPIO_Init(conf->sclk.gpio, &gpio_init_s);

	DMA_DeInit(conf->dma_rx_stream);
	DMA_DeInit(conf->dma_tx_stream);


	// Configure DMA streams
	dma_init_s.DMA_Channel = conf->dma_channel;
	dma_init_s.DMA_PeripheralBaseAddr = (uint32_t) &spi->spi->DR;
	dma_init_s.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_init_s.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_init_s.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma_init_s.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma_init_s.DMA_Mode = DMA_Mode_Normal;
	dma_init_s.DMA_Priority = DMA_Priority_High;
	dma_init_s.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma_init_s.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma_init_s.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma_init_s.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	dma_init_s.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_Init(conf->dma_rx_stream, &dma_init_s);

	dma_init_s.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_Init(conf->dma_tx_stream, &dma_init_s);


	// Configure SPI
	SPI_I2S_DeInit(spi->spi);
	spi_init_s.SPI_Mode = SPI_Mode_Master;
	spi_init_s.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi_init_s.SPI_DataSize = SPI_DataSize_8b;
	spi_init_s.SPI_CPOL = SPI_CPOL_High;
	spi_init_s.SPI_CPHA = SPI_CPHA_2Edge;
	spi_init_s.SPI_NSS = SPI_NSS_Soft;
	spi_init_s.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	spi_init_s.SPI_FirstBit = SPI_FirstBit_MSB;
	spi_init_s.SPI_CRCPolynomial = 7;

	// Enable SPI
	SPI_Init(spi->spi, &spi_init_s);
	SPI_Cmd(spi->spi, ENABLE);

	SPI_DMACmd(spi->spi, SPI_DMAReq_Rx, ENABLE);
	SPI_DMACmd(spi->spi, SPI_DMAReq_Tx, ENABLE);

	// Configure DMA interrupts
	nvic_init_s.NVIC_IRQChannel = conf->dma_irq;
	nvic_init_s.NVIC_IRQChannelPreemptionPriority = 2;
	nvic_init_s.NVIC_IRQChannelSubPriority = 2;
	nvic_init_s.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init_s);

	// Only interrupt on RX complete for entire transaction
	DMA_ITConfig(conf->dma_rx_stream, DMA_IT_TC, ENABLE);

	spi->is_init = 1;
	__enable_irq();
}

void spi_run_xfer(spi_t *spi, spi_transfer_t *xfer){
	const spi_config_t *conf = spi->config;

	if(xfer->nss){
		xfer->nss->gpio->BSRRH = xfer->nss->pin;
	}

	conf->dma_rx_stream->M0AR = (uint32_t)xfer->read_buff;
	conf->dma_tx_stream->M0AR = (uint32_t)xfer->write_buff;
	conf->dma_rx_stream->NDTR = xfer->count;
	conf->dma_tx_stream->NDTR = xfer->count;

	DMA_Cmd(conf->dma_rx_stream, ENABLE);
	DMA_Cmd(conf->dma_tx_stream, ENABLE);
}

void spi_transfer(spi_t *spi, spi_transfer_t *RESTRICT xfer){
	spi_transfer_t *old_xfer;

	// Clear the 'done' flag for this chain of transfers
	for(old_xfer = xfer; old_xfer != NULL; old_xfer = old_xfer->next){
		old_xfer->done = 0;
	}
	__disable_irq();
	
	old_xfer = spi->xfer;
	if(old_xfer == xfer)
		while(1);
	if(old_xfer != NULL){
		for(;old_xfer->next != NULL; old_xfer = old_xfer->next){
			if(old_xfer == xfer)
				while(1);
		}
		old_xfer->next = xfer;
	} else {
		spi_run_xfer(spi, xfer);	
	}
	__enable_irq();
}

void spi_dma_isr(spi_t *spi){
	const spi_config_t *conf = spi->config;
	if(DMA_GetITStatus(conf->dma_rx_stream, conf->dma_rx_tcif)){
		if(spi->xfer->nss){
			spi->xfer->nss->gpio->BSRRL = spi->xfer->nss->pin;
		}
		DMA_Cmd(conf->dma_tx_stream, DISABLE);
		DMA_Cmd(conf->dma_rx_stream, DISABLE);
		DMA_ClearFlag(conf->dma_rx_stream, conf->dma_rx_tc_flag);
		DMA_ClearFlag(conf->dma_tx_stream, conf->dma_tx_tc_flag);
		DMA_ClearITPendingBit(conf->dma_rx_stream, conf->dma_rx_tcif);

		if(spi->xfer->next){
			spi_run_xfer(spi, spi->xfer->next);
		} else {
			spi->xfer = NULL;
		}
	}

}

#if USE_SPI1
void SPI1_DMA_ISR(void){spi_dma_isr(&spi1);}
#endif

#if USE_SPI2
void SPI2_DMA_ISR(void){spi_dma_isr(&spi1);}
#endif

