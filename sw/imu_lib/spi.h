#ifndef __SPI_H_
#define __SPI_H_

#include "sensor_config.h"

typedef enum {
	SPI_MODE_4WIRE = 0,
	SPI_MODE_3WIRE = 1
} spi_mode_t;


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
 @brief A set of hardware-related constants for each I2C device
 */
typedef struct {
	spi_mode_t mode;
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
void spi_transfer(spi_t *spi, spi_transfer_t *__restrict xfer);

#endif

