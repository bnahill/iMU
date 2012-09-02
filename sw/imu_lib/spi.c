#include "stm32f4xx_conf.h"
#include "spi.h"

void spi_init_slave(gpio_pin_t *pin){
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

	spi_config_t const * const conf = spi->config;
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

	// SCLK
	GPIO_PinAFConfig(conf->sclk.gpio, conf->sclk.pinsrc, conf->af);
	gpio_init_s.GPIO_Pin = conf->sclk.pin;
	GPIO_Init(conf->sclk.gpio, &gpio_init_s);

	if(conf->mode == SPI_MODE_4WIRE){
		// MISO
		GPIO_PinAFConfig(conf->miso.gpio, conf->miso.pinsrc, conf->af);
		gpio_init_s.GPIO_Pin = conf->miso.pin;
		GPIO_Init(conf->miso.gpio, &gpio_init_s);
	}
	
	// MOSI
	GPIO_PinAFConfig(conf->mosi.gpio, conf->mosi.pinsrc, conf->af);
	gpio_init_s.GPIO_Pin = conf->mosi.pin;
	GPIO_Init(conf->mosi.gpio, &gpio_init_s);

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
	dma_init_s.DMA_Priority = DMA_Priority_VeryHigh;
	dma_init_s.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma_init_s.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma_init_s.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma_init_s.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	dma_init_s.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_Init(conf->dma_rx_stream, &dma_init_s);
	dma_init_s.DMA_Priority = DMA_Priority_High;
	dma_init_s.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_Init(conf->dma_tx_stream, &dma_init_s);

	// Configure SPI
	SPI_I2S_DeInit(spi->spi);
	spi_init_s.SPI_Mode = SPI_Mode_Master;
	if(conf->mode == SPI_MODE_4WIRE)
		spi_init_s.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	else
		spi_init_s.SPI_Direction = SPI_Direction_1Line_Tx;
	spi_init_s.SPI_DataSize = SPI_DataSize_8b;
	spi_init_s.SPI_CPOL = SPI_CPOL_High;
	spi_init_s.SPI_CPHA = SPI_CPHA_2Edge;
	spi_init_s.SPI_NSS = SPI_NSS_Soft;
	spi_init_s.SPI_BaudRatePrescaler = conf->prescaler;
	spi_init_s.SPI_FirstBit = SPI_FirstBit_MSB;
	spi_init_s.SPI_CRCPolynomial = 7;

	// Enable SPI
	SPI_Init(spi->spi, &spi_init_s);
	SPI_Cmd(spi->spi, ENABLE);

	SPI_DMACmd(spi->spi, SPI_DMAReq_Rx, ENABLE);
	SPI_DMACmd(spi->spi, SPI_DMAReq_Tx, ENABLE);

	// Configure DMA interrupts
	nvic_init_s.NVIC_IRQChannel = conf->dma_rx_irq;
	nvic_init_s.NVIC_IRQChannelPreemptionPriority = 0;
	nvic_init_s.NVIC_IRQChannelSubPriority = 2;
	nvic_init_s.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init_s);

	if(conf->mode == SPI_MODE_3WIRE){
		// Also need an interrupt for TX complete
		nvic_init_s.NVIC_IRQChannelPreemptionPriority = 3;
		nvic_init_s.NVIC_IRQChannel = conf->irq;
		NVIC_Init(&nvic_init_s);
		
		SPI_I2S_ITConfig(spi->spi, SPI_I2S_IT_RXNE, DISABLE); 
		DMA_ITConfig(conf->dma_tx_stream, DMA_IT_TC, DISABLE);
	}

	spi->is_init = 1;
	__enable_irq();
}

void spi_run_xfer(spi_t *spi, spi_transfer_t *xfer){
	spi_config_t const * const conf = spi->config;

	spi->xfer = xfer;

	if(xfer->nss){
		xfer->nss->gpio->BSRRH = xfer->nss->pin;
	}

	conf->dma_rx_stream->M0AR = (uint32_t)xfer->read_buff;
	conf->dma_tx_stream->M0AR = (uint32_t)xfer->write_buff;
	
	if(conf->mode == SPI_MODE_4WIRE){
		conf->dma_rx_stream->NDTR = xfer->write_count + xfer->read_count;
		conf->dma_tx_stream->NDTR = xfer->write_count + xfer->read_count;

		spi->spi->CR1 |= SPI_CR1_SPE;
		xfer->read_count = 0;
		
		DMA_ITConfig(conf->dma_rx_stream, DMA_IT_TC, ENABLE);
		
		conf->dma_rx_stream->CR |= DMA_SxCR_EN;
	} else {
		conf->dma_rx_stream->M0AR = (uint32_t)xfer->read_buff;
		conf->dma_rx_stream->NDTR = xfer->write_count;
		conf->dma_tx_stream->M0AR = (uint32_t)xfer->write_buff;
		conf->dma_tx_stream->NDTR = xfer->write_count;
		
		// Enable output
		spi->spi->CR1 &= ~SPI_CR1_BIDIMODE;
		spi->spi->CR1 |= SPI_CR1_BIDIOE;
		
		spi->spi->CR1 |= SPI_CR1_SPE;
		
		DMA_ITConfig(conf->dma_rx_stream, DMA_IT_TC, ENABLE);
		conf->dma_rx_stream->CR |= DMA_SxCR_EN;
	}
	
	conf->dma_tx_stream->CR |= DMA_SxCR_EN;
}

void spi_transfer(spi_t *spi, spi_transfer_t *__restrict xfer){
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


inline extern void spi_wait(spi_t *__restrict spi){
	uint16_t initial;
	af_gpio_pin_t const * const pin = &spi->config->sclk;
	int i;
	for(i = 0; i < 2; i++){
		initial = pin->gpio->IDR & pin->pin;
		// Wait for clock transition
		while((initial ^ (pin->gpio->IDR & pin->pin)) == 0);
	}
}


#define __SPI_C_
#include "spi_platform.h"

//////////////
// ISRs!
//////////////

void spi_rx_dma_isr(spi_t *__restrict spi){
	spi_config_t const * const conf = spi->config;
	spi_transfer_t * const xfer = spi->xfer;
	if(DMA_GetITStatus(conf->dma_rx_stream, conf->dma_rx_tcif)){
		conf->dma_rx_stream->CR &= ~DMA_SxCR_EN;
		conf->dma_tx_stream->CR &= ~DMA_SxCR_EN;
		DMA_ClearFlag(conf->dma_tx_stream, conf->dma_tx_tc_flag);
		DMA_ClearFlag(conf->dma_rx_stream, conf->dma_rx_tc_flag);
		DMA_ClearITPendingBit(conf->dma_tx_stream, conf->dma_rx_tcif);
		DMA_ClearITPendingBit(conf->dma_rx_stream, conf->dma_rx_tcif);
		
		if(xfer->read_count == 0){
			// Nothing left to do, finish.
			if(xfer->nss){
				xfer->nss->gpio->BSRRL = xfer->nss->pin;
			}
			xfer->done = 1;
			if(xfer->next){
				spi_run_xfer(spi, xfer->next);
			} else {
				spi->xfer = NULL;
			}
		} else if(conf->mode == SPI_MODE_3WIRE){
			if(xfer->read_count > 1){
				spi->spi->CR1 &= ~SPI_CR1_SPE;
				conf->dma_rx_stream->NDTR = xfer->read_count - 1;
				conf->dma_rx_stream->M0AR = (uint32_t)xfer->read_buff + xfer->write_count;
					
				// Switch directions to read
				spi->spi->CR1 |= SPI_CR1_BIDIMODE;
				spi->spi->CR1 &= ~SPI_CR1_BIDIOE;
				
				spi->spi->CR1 |= SPI_CR1_SPE;
				
				conf->dma_rx_stream->CR |= DMA_SxCR_EN;
				
				// Indicate where to write the last byte when it comes up...
				xfer->write_count += xfer->read_count - 1;
				// Mark that tx part is done
				xfer->read_count = 1;
			} else {
				if((spi->spi->CR1 & SPI_CR1_BIDIMODE) == 0){
					// Not continuing the DMA transfer, starting a new single-byte transfer
					spi->spi->CR1 &= ~SPI_CR1_SPE;
					
					// Switch directions to read
					spi->spi->CR1 |= SPI_CR1_BIDIMODE;
					spi->spi->CR1 &= ~SPI_CR1_BIDIOE;
					
					// Start
					spi->spi->CR1 |= SPI_CR1_SPE;
					
					spi_wait(spi);
				}
				
				SPI_I2S_ClearITPendingBit(spi->spi, SPI_I2S_IT_RXNE);
				SPI_I2S_ITConfig(spi->spi, SPI_I2S_IT_RXNE, ENABLE); 
				
				// Stop!
				spi->spi->CR1 &= ~SPI_CR1_SPE;
			}
		}
	}
}

void spi_rx_isr(spi_t *__restrict spi){
	spi_transfer_t * const xfer = spi->xfer;
	SPI_I2S_ClearITPendingBit(spi->spi, SPI_I2S_IT_RXNE);
	SPI_I2S_ITConfig(spi->spi, SPI_I2S_IT_RXNE, DISABLE);
	
	spi->spi->CR1 &= ~SPI_CR1_BIDIMODE;
	
	// The last byte will be written at the write_count position
	// If the last thing done was a write, this will be the correct position
	// automatically.
	// If there was a DMA read involved, the position will have been altered
	// to be correct.
	xfer->read_buff[xfer->write_count] = spi->spi->DR;
	
	// Nothing left to do, finish.
	if(xfer->nss){
		xfer->nss->gpio->BSRRL = xfer->nss->pin;
	}
	xfer->done = 1;
	if(xfer->next){
		spi_run_xfer(spi, xfer->next);
	} else {
		spi->xfer = NULL;
	}
	
	spi->spi->CR1 |= SPI_CR1_SPE;
}

/*!
 @brief ISR for 3-wire SPI
 @param spi The SPI device
 */
void spi_tx_dma_isr(spi_t *__restrict spi){
	spi_config_t const * const conf = spi->config;
	spi_transfer_t * const xfer = spi->xfer;
	if(DMA_GetITStatus(conf->dma_tx_stream, conf->dma_tx_tcif)){
		conf->dma_rx_stream->CR &= ~DMA_SxCR_EN;
		conf->dma_tx_stream->CR &= ~DMA_SxCR_EN;
		
		DMA_ClearFlag(conf->dma_rx_stream, conf->dma_rx_tc_flag);
		DMA_ClearFlag(conf->dma_tx_stream, conf->dma_tx_tc_flag);
		DMA_ClearITPendingBit(conf->dma_tx_stream, conf->dma_tx_tcif);
		DMA_ClearITPendingBit(conf->dma_rx_stream, conf->dma_rx_tcif);
		
		DMA_ITConfig(conf->dma_tx_stream, DMA_IT_TC, DISABLE);
		spi->spi->CR1 &= ~SPI_CR1_SPE;
		
		
		if(xfer->read_count == 0){
			// Done
			if(spi->xfer->nss){
				spi->xfer->nss->gpio->BSRRL = spi->xfer->nss->pin;
			}
			spi->spi->CR1 |= SPI_CR1_BIDIMODE;
			spi->xfer->done = 1;
			if(spi->xfer->next){
				spi_run_xfer(spi, spi->xfer->next);
			} else {
				spi->xfer = NULL;
			}
			return;
		}
		
		DMA_ITConfig(conf->dma_rx_stream, DMA_IT_TC, ENABLE);
		
		conf->dma_rx_stream->NDTR = spi->xfer->read_count;
		conf->dma_tx_stream->NDTR = spi->xfer->read_count;
		conf->dma_rx_stream->M0AR = (uint32_t)spi->xfer->read_buff + spi->xfer->write_count;
		
		
		
		// Switch directions to read
		spi->spi->CR1 |= SPI_CR1_BIDIMODE;
		spi->spi->CR1 &= ~SPI_CR1_BIDIOE;
		
		
		spi->spi->CR1 |= SPI_CR1_SPE;
	
		// Mark that tx part is done
		xfer->read_count = 0;
		
		conf->dma_rx_stream->CR |= DMA_SxCR_EN;
		conf->dma_tx_stream->CR |= DMA_SxCR_EN;
	}
}


