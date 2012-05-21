#ifndef __SDIO_LL_H_
#define __SDIO_LL_H_

#include "stm32f4xx_conf.h"
#include "sensor_config.h"

#define SDIO_INIT_CLK_DIV ((uint8_t)0x76)
#define SDIO_TRANSFER_CLK_DIV ((uint8_t)0x00)

#define SD_DETECT_GPIO_PORT GPIOD
#define SD_DETECT_PIN  BIT(3)

#define DMA_CLK      RCC_AHB1Periph_DMA2

#define SD_SDIO_DMA_STREAM    DMA2_Stream6
#define DMA_CHANNEL           DMA_Channel_4
#define SD_SDIO_DMA_FLAG_FEIF DMA_FLAG_FEIF6
#define DMA_FLAG_DMEIF        DMA_FLAG_DMEIF6
#define DMA_FLAG_TEIF         DMA_FLAG_TEIF6
#define DMA_FLAG_HTIF         DMA_FLAG_HTIF6
#define SD_SDIO_DMA_FLAG_TCIF DMA_FLAG_TCIF6 
#define DMA_IRQn              DMA2_Stream6_IRQn
#define DMA_IRQHANDLER        DMA2_Stream6_IRQHandler
#define FIFO_ADDRESS          ((uint32_t)0x40012C80)

void SD_LowLevel_Init(void);
void SD_LowLevel_DeInit(void);
void SD_LowLevel_DMA_RxConfig(uint32_t *dst, uint32_t cnt);
void SD_LowLevel_DMA_TxConfig(uint32_t *src, uint32_t cnt);

#endif

