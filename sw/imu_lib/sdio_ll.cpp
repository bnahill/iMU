#include "sdio_ll.h"

void SD_LowLevel_Init(void){
	GPIO_InitTypeDef  gpio_init_s;
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_SDIO);

	GPIO_StructInit(&gpio_init_s);

	// Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins
	gpio_init_s.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	gpio_init_s.GPIO_Speed = GPIO_Speed_25MHz;
	gpio_init_s.GPIO_Mode = GPIO_Mode_AF;
	gpio_init_s.GPIO_OType = GPIO_OType_PP;
	gpio_init_s.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &gpio_init_s);

	// Configure PD.02 CMD line
	gpio_init_s.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &gpio_init_s);

	// Configure PC.12 pin: CLK pin
	gpio_init_s.GPIO_Pin = GPIO_Pin_12;
	gpio_init_s.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &gpio_init_s);

	// Configure SD Card detect pin
	gpio_init_s.GPIO_Pin = SD_DETECT_PIN;
	gpio_init_s.GPIO_Mode = GPIO_Mode_IN;
	gpio_init_s.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SD_DETECT_GPIO_PORT, &gpio_init_s);

	// Enable the SDIO APB2 Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);

	// Enable the DMA2 Clock
	RCC_AHB1PeriphClockCmd(DMA_CLK, ENABLE);
}

void SD_LowLevel_DeInit(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

	/*!< Disable SDIO Clock */
	SDIO_ClockCmd(DISABLE);

	/*!< Set Power State to OFF */
	SDIO_SetPowerState(SDIO_PowerState_OFF);

	/*!< DeInitializes the SDIO peripheral */
	SDIO_DeInit();

	/* Disable the SDIO APB2 Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, DISABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_MCO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_MCO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_MCO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_MCO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_MCO);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_MCO);

	/* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure PD.02 CMD line */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Configure PC.12 pin: CLK pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void SD_LowLevel_DMA_RxConfig(uint32_t *dst, uint32_t cnt){
	DMA_InitTypeDef dma_init_s;

	DMA_ClearFlag(SD_SDIO_DMA_STREAM, SD_SDIO_DMA_FLAG_FEIF | DMA_FLAG_DMEIF | DMA_FLAG_TEIF | DMA_FLAG_HTIF | SD_SDIO_DMA_FLAG_TCIF);

	DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);

	DMA_DeInit(SD_SDIO_DMA_STREAM);

	dma_init_s.DMA_Channel = DMA_CHANNEL;
	dma_init_s.DMA_PeripheralBaseAddr = (uint32_t)FIFO_ADDRESS;
	dma_init_s.DMA_Memory0BaseAddr = (uint32_t)dst;
	dma_init_s.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma_init_s.DMA_BufferSize = 0;
	dma_init_s.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_init_s.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_init_s.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	dma_init_s.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	dma_init_s.DMA_Mode = DMA_Mode_Normal;
	dma_init_s.DMA_Priority = DMA_Priority_VeryHigh;
	dma_init_s.DMA_FIFOMode = DMA_FIFOMode_Enable;
	dma_init_s.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma_init_s.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
	dma_init_s.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
	
	DMA_Init(SD_SDIO_DMA_STREAM, &dma_init_s);
	DMA_ITConfig(SD_SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
	DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

	/* DMA2 Stream3 or Stream6 enable */
	DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
}

void SD_LowLevel_DMA_TxConfig(uint32_t *src, uint32_t cnt){
	DMA_InitTypeDef dma_init_s;

	DMA_ClearFlag(SD_SDIO_DMA_STREAM, SD_SDIO_DMA_FLAG_FEIF | DMA_FLAG_DMEIF | DMA_FLAG_TEIF | DMA_FLAG_HTIF | SD_SDIO_DMA_FLAG_TCIF);

	DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);

	DMA_DeInit(SD_SDIO_DMA_STREAM);

	dma_init_s.DMA_Channel = DMA_CHANNEL;
	dma_init_s.DMA_PeripheralBaseAddr = (uint32_t)FIFO_ADDRESS;
	dma_init_s.DMA_Memory0BaseAddr = (uint32_t)src;
	dma_init_s.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dma_init_s.DMA_BufferSize = 0;
	dma_init_s.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_init_s.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_init_s.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	dma_init_s.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	dma_init_s.DMA_Mode = DMA_Mode_Normal;
	dma_init_s.DMA_Priority = DMA_Priority_VeryHigh;
	dma_init_s.DMA_FIFOMode = DMA_FIFOMode_Enable;
	dma_init_s.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma_init_s.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
	dma_init_s.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
	DMA_Init(SD_SDIO_DMA_STREAM, &dma_init_s);
	DMA_ITConfig(SD_SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
	DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

	DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
}


