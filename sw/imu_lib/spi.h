#ifndef __SPI_H_
#define __SPI_H_

#include "sensor_config.h"

typedef struct _spi_transfer {
	//! The data from the slave
	uint8_t *read_buff;
	//! The data to be written to the slave
	uint8_t *write_buff;
	//! The NSS pin to use, may be null
	gpio_pin_t const * nss;
	//! The number of bytes to transfer
	uint16_t count;
	//! A flag to indicate completion
	uint8_t done;
	//! The next transfer in the queue
	struct _spi_transfer *next;
} spi_transfer_t;

/*!
 @brief A set of hardware-related constants for each I2C device
 */
typedef struct {
	uint8_t af;
	af_gpio_pin_t miso;
	af_gpio_pin_t mosi;
	af_gpio_pin_t sclk;
	DMA_Stream_TypeDef *dma_rx_stream;
	DMA_Stream_TypeDef *dma_tx_stream;
	DMA_TypeDef        *dma;
	uint32_t dma_channel;
	uint32_t dma_rx_tcif;
	uint32_t dma_rx_tc_flag;
	uint32_t dma_tx_tc_flag;
	IRQn_Type dma_irq;
	uint32_t dma_clock;
	void (*clock_cmd)(uint32_t, FunctionalState);
	uint32_t clock;
} spi_config_t;

/*!
 @brief The state and configuration of an SPI device
 */
typedef struct {
	//! The SPI device
	SPI_TypeDef * const spi;
	//! The current transfer
	spi_transfer_t *xfer;
	//! A flag to indicate that the device is already configured
	uint8_t is_init;
	//! A pointer to the hardware configuration
	spi_config_t const * const config;
} spi_t;

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

/*!
 @brief Begin a SPI transfer
 @param spi The SPI device to use
 @param xfer The transfer structure to use

 This will start a new SPI transfer for the device provided. If the device is
 busy, the transfer will be added to the transfer queue. To check for
 completion, the caller is expected to wait on the spi_transfer_t 'done' flag.
 */
void spi_transfer(spi_t *spi, spi_transfer_t *RESTRICT xfer);

#if USE_SPI1
extern spi_t spi1;
#endif

#if USE_I2C2
extern spi_t spi2;
#endif


#endif

