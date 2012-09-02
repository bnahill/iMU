#ifndef __SPI_PRIVATE_H_
#define __SPI_PRIVATE_H_

#include "spi.h"

void spi_rx_isr(spi_t *__restrict spi);
void spi_rx_dma_isr(spi_t *__restrict spi);
void spi_tx_dma_isr(spi_t *__restrict spi);

#endif
