#ifndef _SPI_PUBLIC_H__
#define _SPI_PUBLIC_H__

#include "spi.h"

#if USE_SPI1
extern spi_t spi1;
#endif

#if USE_I2C2
extern spi_t spi2;
#endif

#endif
