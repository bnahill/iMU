#ifndef __SPI_H_
#define __SPI_H_

#include "sensor_config.h"


typedef struct _spi_transfer {
	//! The data from the slave; Must be read_count + write_count bytes
	uint8_t *read_buff;
	//! The data to be written to the slave; Must be read_count + write_count bytes
	uint8_t *write_buff;
	//! The NSS pin to use, may be null
	gpio_pin_t const * nss;
	//! The number of bytes to write (no read when using 3-wire)
	uint16_t write_count;
	//! The number of bytes to read after writing
	uint16_t read_count;
	//! A flag to indicate completion
	uint8_t done;
	//! The next transfer in the queue
	struct _spi_transfer *next;
} spi_transfer_t;

/*!
 The parent SPI class defining the interface and common operations for 3-
 and 4-wire SPI
 @template configtype The type of configuration struct used here
 @note The template parameter is so that the common code may read config
 values
 */
template <typename configtype> class SPI {
public:
	SPI(configtype const &_config, SPI_TypeDef *_spi) :
		spi(spi), config(_config){}
	
	bool init();
	void run_xfer(spi_transfer_t *xfer);
	bool is_init;
	
	spi_transfer_t *xfer;
	
	static void mk_transfer(spi_transfer_t &xfer, uint8_t *r_buff, uint8_t *w_buff, gpio_pin_t const &nss, uint16_t w_len, uint16_t r_len){
		xfer.read_buff = r_buff;
		xfer.write_buff = w_buff;
		xfer.write_count = w_len;
		xfer.read_count = r_len;
		xfer.nss = &nss;
		xfer.next = nullptr;
	}
	
	static void slave_init(gpio_pin_t const &pin){
		GPIO_InitTypeDef gpio_init_s;
		
		GPIO_StructInit(&gpio_init_s);
		
		gpio_init_s.GPIO_Speed = GPIO_Speed_100MHz;
		gpio_init_s.GPIO_Mode = GPIO_Mode_OUT;
		gpio_init_s.GPIO_OType = GPIO_OType_PP;
		gpio_init_s.GPIO_Pin = pin.pin;

		pin.gpio->BSRRL = pin.pin;
		
		GPIO_Init(pin.gpio, &gpio_init_s);
	}
	
	/*!
	 Add a transfer to the queue
	 */
	void transfer(spi_transfer_t * __restrict _xfer){
		spi_transfer_t *old_xfer;

		// Clear the 'done' flag for this chain of transfers
		for(old_xfer = _xfer; old_xfer != NULL; old_xfer = old_xfer->next){
			old_xfer->done = 0;
		}
		__disable_irq();
		
		old_xfer = xfer;
		if(old_xfer == _xfer){
			// This transfer is already in the queue...
			return;
		}
		if(old_xfer != NULL){
			for(;old_xfer->next != NULL; old_xfer = old_xfer->next){
				if(old_xfer == xfer)
					while(1);
			}
			old_xfer->next = xfer;
		} else {
			run_xfer(xfer);	
		}
		
		__enable_irq();
	}
	
	SPI_TypeDef * const spi;
protected:
	configtype const config;
};

typedef struct {
	uint8_t af;
	af_gpio_pin_t miso;
	af_gpio_pin_t mosi;
	af_gpio_pin_t sclk;
	uint16_t prescaler;
	DMA_Stream_TypeDef *dma_rx_stream;
	DMA_Stream_TypeDef *dma_tx_stream;
	DMA_TypeDef        *dma;
	uint32_t dma_channel;
	uint32_t dma_rx_tcif;
	uint32_t dma_rx_tc_flag;
	uint32_t dma_tx_tc_flag;
	IRQn_Type dma_irq;
	uint32_t dma_clock;
	IRQn_Type irq;
	void (*clock_cmd)(uint32_t, FunctionalState);
	uint32_t clock;
} SPI_4WireConfig;

typedef struct {
	uint8_t af;
	af_gpio_pin_t miso;
	af_gpio_pin_t mosi;
	af_gpio_pin_t sclk;
	uint16_t prescaler;
	DMA_Stream_TypeDef *dma_rx_stream;
	DMA_Stream_TypeDef *dma_tx_stream;
	DMA_TypeDef        *dma;
	uint32_t dma_channel;
	uint32_t dma_rx_tcif;
	uint32_t dma_tx_tcif;
	uint32_t dma_rx_tc_flag;
	uint32_t dma_tx_tc_flag;
	IRQn_Type dma_rx_irq;
	IRQn_Type dma_tx_irq;
	uint32_t dma_clock;
	IRQn_Type irq;
	void (*clock_cmd)(uint32_t, FunctionalState);
	uint32_t clock;
} SPI_3WireConfig;


/*!
 A traditional 4-wire SPI interface
 */
class SPI_4Wire : public SPI <SPI_4WireConfig> {
public:
	SPI_4Wire(SPI_4WireConfig const &_config, SPI_TypeDef *_spi) :
	SPI<SPI_4WireConfig>(_config, _spi){}
	
	bool init();
	
	void isr_dma_rx(){
		if(DMA_GetITStatus(config.dma_rx_stream, config.dma_rx_tcif)){
			config.dma_rx_stream->CR &= ~DMA_SxCR_EN;
			config.dma_tx_stream->CR &= ~DMA_SxCR_EN;
			DMA_ClearFlag(config.dma_tx_stream, config.dma_tx_tc_flag);
			DMA_ClearFlag(config.dma_rx_stream, config.dma_rx_tc_flag);
			DMA_ClearITPendingBit(config.dma_tx_stream, config.dma_rx_tcif);
			DMA_ClearITPendingBit(config.dma_rx_stream, config.dma_rx_tcif);
			
			if(xfer->read_count == 0){
				// Nothing left to do, finish.
				if(xfer->nss){
					xfer->nss->gpio->BSRRL = xfer->nss->pin;
				}
				xfer->done = 1;
				if(xfer->next){
					run_xfer(xfer->next);
				} else {
					xfer = nullptr;
				}
			}
		}	
	}
private:
	void run_xfer(spi_transfer_t *xfer);
	
	
	
};

/*!
 A 3-wire SPI interface where MISO and MOSI are multiplexed to just use MOSI
 */
class SPI_3Wire : public SPI <SPI_3WireConfig> {
public:
	SPI_3Wire(SPI_3WireConfig const &_config, SPI_TypeDef *_spi) :
	SPI<SPI_3WireConfig>(_config, _spi){}
	
	bool init();
	
	void isr_spi_rx(){
		SPI_I2S_ClearITPendingBit(spi, SPI_I2S_IT_RXNE);
		SPI_I2S_ITConfig(spi, SPI_I2S_IT_RXNE, DISABLE);
		
		spi->CR1 &= ~SPI_CR1_BIDIMODE;
		
		// The last byte will be written at the write_count position
		// If the last thing done was a write, this will be the correct position
		// automatically.
		// If there was a DMA read involved, the position will have been altered
		// to be correct.
		xfer->read_buff[xfer->write_count] = spi->DR;
		
		// Nothing left to do, finish.
		if(xfer->nss){
			xfer->nss->gpio->BSRRL = xfer->nss->pin;
		}
		xfer->done = 1;
		if(xfer->next){
			run_xfer(xfer->next);
		} else {
			xfer = nullptr;
		}
		
		spi->CR1 |= SPI_CR1_SPE;	
	}
	void isr_dma_tx(){
		if(DMA_GetITStatus(config.dma_tx_stream, config.dma_tx_tcif)){
			config.dma_rx_stream->CR &= ~DMA_SxCR_EN;
			config.dma_tx_stream->CR &= ~DMA_SxCR_EN;
			
			DMA_ClearFlag(config.dma_rx_stream, config.dma_rx_tc_flag);
			DMA_ClearFlag(config.dma_tx_stream, config.dma_tx_tc_flag);
			DMA_ClearITPendingBit(config.dma_tx_stream, config.dma_tx_tcif);
			DMA_ClearITPendingBit(config.dma_rx_stream, config.dma_rx_tcif);
			
			DMA_ITConfig(config.dma_tx_stream, DMA_IT_TC, DISABLE);
			spi->CR1 &= ~SPI_CR1_SPE;
			
			
			if(xfer->read_count == 0){
				// Done
				if(xfer->nss){
					xfer->nss->gpio->BSRRL = xfer->nss->pin;
				}
				spi->CR1 |= SPI_CR1_BIDIMODE;
				xfer->done = 1;
				if(xfer->next){
					run_xfer(xfer->next);
				} else {
					xfer = nullptr;
				}
				return;
			}
			
			DMA_ITConfig(config.dma_rx_stream, DMA_IT_TC, ENABLE);
			
			config.dma_rx_stream->NDTR = xfer->read_count;
			config.dma_tx_stream->NDTR = xfer->read_count;
			config.dma_rx_stream->M0AR = (uint32_t)xfer->read_buff + xfer->write_count;
			
			// Switch directions to read
			spi->CR1 |= SPI_CR1_BIDIMODE;
			spi->CR1 &= ~SPI_CR1_BIDIOE;
			
			
			spi->CR1 |= SPI_CR1_SPE;
		
			// Mark that tx part is done
			xfer->read_count = 0;
			
			config.dma_rx_stream->CR |= DMA_SxCR_EN;
			config.dma_tx_stream->CR |= DMA_SxCR_EN;
		}
	}
	
	void isr_dma_rx(){
		if(DMA_GetITStatus(config.dma_rx_stream, config.dma_rx_tcif)){
			config.dma_rx_stream->CR &= ~DMA_SxCR_EN;
			config.dma_tx_stream->CR &= ~DMA_SxCR_EN;
			DMA_ClearFlag(config.dma_tx_stream, config.dma_tx_tc_flag);
			DMA_ClearFlag(config.dma_rx_stream, config.dma_rx_tc_flag);
			DMA_ClearITPendingBit(config.dma_tx_stream, config.dma_rx_tcif);
			DMA_ClearITPendingBit(config.dma_rx_stream, config.dma_rx_tcif);
			
			if(xfer->read_count == 0){
				// Nothing left to do, finish.
				if(xfer->nss){
					xfer->nss->gpio->BSRRL = xfer->nss->pin;
				}
				xfer->done = 1;
				if(xfer->next){
					run_xfer(xfer->next);
				} else {
					xfer = nullptr;
				}
			} else {
				if(xfer->read_count > 1){
					spi->CR1 &= ~SPI_CR1_SPE;
					config.dma_rx_stream->NDTR = xfer->read_count - 1;
					config.dma_rx_stream->M0AR = (uint32_t)xfer->read_buff + xfer->write_count;
						
					// Switch directions to read
					spi->CR1 |= SPI_CR1_BIDIMODE;
					spi->CR1 &= ~SPI_CR1_BIDIOE;
					
					spi->CR1 |= SPI_CR1_SPE;
					
					config.dma_rx_stream->CR |= DMA_SxCR_EN;
					
					// Indicate where to write the last byte when it comes up...
					xfer->write_count += xfer->read_count - 1;
					// Mark that tx part is done
					xfer->read_count = 1;
				} else {
					if((spi->CR1 & SPI_CR1_BIDIMODE) == 0){
						// Not continuing the DMA transfer, starting a new single-byte transfer
						spi->CR1 &= ~SPI_CR1_SPE;
						
						// Switch directions to read
						spi->CR1 |= SPI_CR1_BIDIMODE;
						spi->CR1 &= ~SPI_CR1_BIDIOE;
						
						// Start
						spi->CR1 |= SPI_CR1_SPE;
						
						wait();
					}
					
					SPI_I2S_ClearITPendingBit(spi, SPI_I2S_IT_RXNE);
					SPI_I2S_ITConfig(spi, SPI_I2S_IT_RXNE, ENABLE); 
					
					// Stop!
					spi->CR1 &= ~SPI_CR1_SPE;
				}
			}
		}	
	}
private:
	void run_xfer(spi_transfer_t *xfer);
	void wait(){
		uint16_t initial;
		int i;
		for(i = 0; i < 2; i++){
			initial = config.sclk.gpio->IDR & config.sclk.pin;
			// Wait for clock transition
			while((initial ^ (config.sclk.gpio->IDR & config.sclk.pin)) == 0);
		}
	}
};

/*!
 @brief The state and configuration of an SPI device
 */

#if 0
/*!
 @brief Initialize a SPI peripheral
 @param spi The SPI peripheral to initialize
 */
void spi_init(spi_t *spi);

/*!
 @brief Configure a GPIO pin for use to control a slave device
 @param pin The pin to configure
 */
void spi_init_slave(gpio_pin_t *pin);


void spi_transfer(spi_t *spi, spi_transfer_t *__restrict xfer);
#endif

#endif

