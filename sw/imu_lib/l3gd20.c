#include "stm32f4xx_conf.h"

#include "l3gd20.h"
#include "sensor_config.h"

#define BIT(x) (1 << x)

#define GYRO_1_CS_GPIO    GPIOC
#define GYRO_1_CS_PIN     BIT(1)

#define GYRO_2_CS_GPIO    GPIOE
#define GYRO_2_CS_PIN     BIT(15)

#define GYRO_SCK_GPIO     GPIOA
#define GYRO_SCK_PIN      BIT(5)
#define GYRO_SCK_PINSRC   GPIO_PinSource5
#define GYRO_MISO_GPIO    GPIOB
#define GYRO_MISO_PIN     BIT(4)
#define GYRO_MISO_PINSRC  GPIO_PinSource4
#define GYRO_MOSI_GPIO    GPIOB
#define GYRO_MOSI_PIN     BIT(5)
#define GYRO_MOSI_PINSRC  GPIO_PinSource5

#define GYRO_SPI          SPI1
#define GYRO_SPI_CLK_CMD  RCC_APB2PeriphClockCmd
#define GYRO_SPI_CLK      RCC_APB2Periph_SPI1
#define GYRO_SPI_AF       GPIO_AF_SPI1

#define GYRO_DMA            DMA2
#define GYRO_DMA_CHANNEL    DMA_Channel_3
#define GYRO_DMA_RX_STREAM  DMA2_Stream0
#define GYRO_DMA_TX_STREAM  DMA2_Stream2
#define GYRO_DMA_RX_ISR     DMA2_Stream0_IRQHandler
#define GYRO_DMA_RX_IRQ     DMA2_Stream0_IRQn
#define GYRO_DMA_RX_TCIF    DMA_IT_TCIF0


#define GYRO_REG_WHO_AM_I     0x0F
#define GYRO_REG_CTRL_REG1    0x20
#define GYRO_REG_CTRL_REG2    0x21
#define GYRO_REG_CTRL_REG3    0x22
#define GYRO_REG_CTRL_REG4    0x23
#define GYRO_REG_CTRL_REG5    0x24
#define GYRO_REG_REF_DATACAP  0x25
#define GYRO_REG_OUT_TEMP     0x26
#define GYRO_REG_STATUS       0x27
#define GYRO_REG_OUT_X_L      0x28
#define GYRO_REG_OUT_X_H      0x29
#define GYRO_REG_OUT_Y_L      0x2A
#define GYRO_REG_OUT_Y_H      0x2B
#define GYRO_REG_OUT_Z_L      0x2C
#define GYRO_REG_OUT_Z_H      0x2D
#define GYRO_REG_FIFO_CTRL    0x2E
#define GYRO_REG_FIFO_SRC     0x2F
#define GYRO_REG_INT1_CFG     0x30
#define GYRO_REG_INT1_SRC     0x31
#define GYRO_REG_INT1_THS_XH  0x32
#define GYRO_REG_INT1_THS_XL  0x33
#define GYRO_REG_INT1_THS_YH  0x34
#define GYRO_REG_INT1_THS_YL  0x35
#define GYRO_REG_INT1_THS_ZH  0x36
#define GYRO_REG_INT1_THS_ZL  0x37
#define GYRO_REG_INT1_DURA    0x38


#if HAS_GYRO_1
euclidean3_t gyro1_result;
#endif

#if HAS_GYRO_2
euclidean3_t gyro2_result;
#endif

static uint8_t l3gd20_read_register(uint8_t addr);
static void l3gd20_transfer_dma(void *restrict r_buff, void *restrict w_buff, uint16_t len);

void l3gd20_init(void){
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
#if GYRO_USE_DMA
	DMA_InitTypeDef  DMA_InitStructure;
#endif

	GYRO_SPI_CLK_CMD(GYRO_SPI_CLK, ENABLE);

	// Configure CS signals
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
#if HAS_GYRO_1
	GPIO_InitStructure.GPIO_Pin = GYRO_1_CS_PIN;
	GPIO_Init(GYRO_1_CS_GPIO, &GPIO_InitStructure);
#endif
#if HAS_GYRO_2
	GPIO_InitStructure.GPIO_Pin = GYRO_2_CS_PIN;
	GPIO_Init(GYRO_2_CS_GPIO, &GPIO_InitStructure);
#endif


	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	// SCK
	GPIO_PinAFConfig(GYRO_SCK_GPIO, GYRO_SCK_PINSRC, GYRO_SPI_AF);
	GPIO_InitStructure.GPIO_Pin = GYRO_SCK_PIN;
	GPIO_Init(GYRO_SCK_GPIO, &GPIO_InitStructure);
	// MISO
	GPIO_PinAFConfig(GYRO_MISO_GPIO, GYRO_SCK_PINSRC, GYRO_SPI_AF);
	GPIO_InitStructure.GPIO_Pin = GYRO_MISO_PIN;
	GPIO_Init(GYRO_MISO_GPIO, &GPIO_InitStructure);
	// MOSI
	GPIO_PinAFConfig(GYRO_MOSI_GPIO, GYRO_MOSI_PINSRC, GYRO_SPI_AF);
	GPIO_InitStructure.GPIO_Pin = GYRO_MOSI_PIN;
	GPIO_Init(GYRO_MOSI_GPIO, &GPIO_InitStructure);

#if GYRO_USE_DMA
	DMA_DeInit(GYRO_DMA_RX_STREAM);
	DMA_DeInit(GYRO_DMA_TX_STREAM);

	DMA_InitStructure.DMA_Channel = GYRO_DMA_CHANNEL;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(GYRO_DMA_RX_STREAM, &DMA_InitStructure);

	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_Init(GYRO_DMA_TX_STREAM, &DMA_InitStructure);
#endif


	SPI_I2S_DeInit(GYRO_SPI);
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

#if GYRO_USE_DMA
	SPI_DMACmd(GYRO_SPI, SPI_DMAReq_Rx | SPI_DMAReq_Tx, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = GYRO_DMA_RX_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Only interrupt on RX complete for entire transaction
	DMA_ITConfig(GYRO_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
#endif
	SPI_Init(GYRO_SPI, &SPI_InitStructure);

#if HAS_GYRO_1
	GYRO_1_CS_GPIO->ODR &= ~GYRO_1_CS_PIN;
	if(l3gd20_read_register(GYRO_REG_WHO_AM_I) != 0xD4)
		while(1);
	GYRO_1_CS_GPIO->ODR |= GYRO_1_CS_PIN;
#endif

#if HAS_GYRO_2
	GYRO_2_CS_GPIO->ODR &= ~GYRO_2_CS_PIN;
	if(l3gd20_read_register(GYRO_REG_WHO_AM_I) != 0xD4)
		while(1);
	GYRO_2_CS_GPIO->ODR |= GYRO_2_CS_PIN;
#endif
}

static uint8_t l3gd20_read_register(uint8_t addr){
	uint8_t read_buff[2], write_buff[2];
	write_buff[0] = addr | 0x80;
	l3gd20_transfer_dma(read_buff, write_buff, 2);
}

void l3gd20_read(void){
}

#if GYRO_USE_DMA
static void l3gd20_transfer_dma(void *restrict r_buff, void *restrict w_buff, uint16_t len){
	GYRO_DMA_RX_STREAM->M0AR = (uint32_t)r_buff;
	GYRO_DMA_TX_STREAM->M0AR = (uint32_t)r_buff;
	GYRO_DMA_RX_STREAM->NDTR = len;
	GYRO_DMA_TX_STREAM->NDTR = len;

	DMA_Cmd(GYRO_DMA_RX_STREAM, ENABLE);
}
#endif

void GYRO_DMA_RX_ISR(void){
	if(DMA_GetITStatus(GYRO_DMA_RX_STREAM, GYRO_DMA_RX_TCIF)){
	    DMA_ClearITPendingBit(GYRO_DMA_RX_STREAM, GYRO_DMA_RX_TCIF);
	}
}
