#include "stm32f4xx_conf.h"

#include "l3gd20.h"
#include "spi.h"
#include "sensor_config.h"

#define GYRO_FLAG_READ      0x80
#define GYRO_FLAG_WRITE     0x00
#define GYRO_FLAG_SEQ       0x40
#define GYRO_FLAG_NOSEQ     0x00

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

#define GYRO_SCALE_250_DPS    (8.75/1000.0)
#define GYRO_SCALE_500_DPS    (17.50/1000.0)
#define GYRO_SCALE_2000_DPS   (70.0/1000.0)

#if HAS_GYRO_1
l3gd20_t gyro1 = {
	.reading = {0.0, 0.0, 0.0},
	.nss = {GPIOC, BIT(1)},
	.dps_scale = GYRO_SCALE_500_DPS,
	.spi = &spi1
};
#endif

#if HAS_GYRO_2
l3gd20_t gyro2 = {
	.reading = {0.0, 0.0, 0.0},
	.nss = {GPIOE, BIT(15)},
	.dps_scale = GYRO_SCALE_500_DPS,
	.spi = &spi1
};
#endif

//! A buffer for the command to read values from a gyroscope
static uint8_t gyro_write_buffer[7];

//! @name Private
//! @{
static uint8_t l3gd20_read_register(l3gd20_t *RESTRICT gyro, uint8_t addr);
static void l3gd20_configure_device(l3gd20_t *RESTRICT gyro);
static void l3gd20_device_init(l3gd20_t *gyro);
static void l3gd20_write_register(l3gd20_t *RESTRICT gyro, uint8_t addr, uint8_t value);
static void l3gd20_transfer_sync(l3gd20_t *gyro, uint8_t *RESTRICT r_buff, uint8_t *RESTRICT w_buff, uint16_t len);
static void l3gd20_read_sensor(l3gd20_t *RESTRICT gyro);
static void l3gd20_update_device(l3gd20_t * const gyro);
static volatile int is_done;
// @}

//////////////////////////////////////////////////////////////////////////////
// Public functions
//////////////////////////////////////////////////////////////////////////////

void l3gd20_init(void){
#if HAS_GYRO_1
	spi_init_slave(&gyro1.nss);
#endif
#if HAS_GYRO_2
	spi_init_slave(&gyro2.nss);
#endif
#if HAS_GYRO_1
	l3gd20_device_init(&gyro1);
#endif
#if HAS_GYRO_2
	l3gd20_device_init(&gyro2);
#endif
}

void l3gd20_read_sync(void){
	l3gd20_read();
	while(!l3gd20_transfer_complete());
	l3gd20_update();
}

void l3gd20_read(void){
#if HAS_GYRO_1
	l3gd20_read_sensor(&gyro1);
#endif
#if HAS_GYRO_2
	l3gd20_read_sensor(&gyro2);
#endif
}
void l3gd20_update(void){
#if HAS_GYRO_1
	l3gd20_update_device(&gyro1);
#endif
#if HAS_GYRO_2
	l3gd20_update_device(&gyro2);
#endif
}

uint8_t l3gd20_transfer_complete(void){
	uint8_t ret = 1;
	
	#if HAS_GYRO_1
		ret &= gyro1.xfer.done;
	#endif
	#if HAS_GYRO_2
		ret &= gyro2.xfer.done;
	#endif
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
// Private support functions
//////////////////////////////////////////////////////////////////////////////

static void l3gd20_configure_device(l3gd20_t *RESTRICT gyro){
	if(l3gd20_read_register(gyro, GYRO_REG_WHO_AM_I) != 0xD4)
		while(1);
	if(l3gd20_read_register(gyro, GYRO_REG_WHO_AM_I) != 0xD4)
		while(1);
	// Set for 190Hz with 70Hz BW, normal mode with all axes on
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG1, 0b01111111);
	if(l3gd20_read_register(gyro, GYRO_REG_CTRL_REG1) != 0b01111111)
		while(1);
	// Set 0.018Hz HP
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG2, 0b00001001);
	if(l3gd20_read_register(gyro, GYRO_REG_CTRL_REG2) != 0b00001001)
		while(1);
	// Disable all interrupts
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG3, 0x00);
	// Enable BDU (delay 1 cycle), little endian, 500dps, 4-wire SPI
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG4, 0b10010000);
	if(l3gd20_read_register(gyro, GYRO_REG_CTRL_REG4) != 0b10010000)
		while(1);
	// Enable HP filter and use it
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG5, 0b00010011);
	if(l3gd20_read_register(gyro, GYRO_REG_CTRL_REG5) != 0b00010011)
		while(1);
	// FIFO bypass mode
	l3gd20_write_register(gyro, GYRO_REG_FIFO_CTRL, 0x00);
}

static void l3gd20_device_init(l3gd20_t *gyro){
	spi_init(gyro->spi);
	l3gd20_configure_device(gyro);
}


static void l3gd20_write_register(l3gd20_t *RESTRICT gyro, uint8_t addr, uint8_t value){
	uint8_t read_buff[2], write_buff[2];
	write_buff[0] = GYRO_FLAG_WRITE | GYRO_FLAG_NOSEQ | addr;
	write_buff[1] = value;
	l3gd20_transfer_sync(gyro, read_buff, write_buff, 2);
}

static uint8_t l3gd20_read_register(l3gd20_t *RESTRICT gyro, uint8_t addr){
	uint8_t read_buff[2], write_buff[2];
	write_buff[0] = GYRO_FLAG_READ | GYRO_FLAG_NOSEQ | addr;
	write_buff[1] = 0;
	l3gd20_transfer_sync(gyro, read_buff, write_buff, 2);
	return read_buff[1];
}

static void l3gd20_read_sensor(l3gd20_t *RESTRICT gyro){
	int16_t tmp;

	gyro_write_buffer[0] = GYRO_FLAG_READ | GYRO_FLAG_SEQ | GYRO_REG_OUT_X_L;
	
	gyro->xfer.read_buff = gyro->r_buff;
	gyro->xfer.write_buff = gyro_write_buffer;
	gyro->xfer.nss = &gyro->nss;
	gyro->xfer.count = 7;
	gyro->xfer.next = NULL;

	spi_transfer(gyro->spi, &gyro->xfer);
}



static void l3gd20_update_device(l3gd20_t * const gyro){
	int16_t tmp;
	tmp = (gyro->r_buff[2] << 8) | gyro->r_buff[1];
	gyro->reading.x = tmp * gyro->dps_scale;
	tmp = (gyro->r_buff[4] << 8) | gyro->r_buff[3];
	gyro->reading.y = tmp * gyro->dps_scale;
	tmp = (gyro->r_buff[6] << 8) | gyro->r_buff[5];
	gyro->reading.z = tmp * gyro->dps_scale;
}

static void l3gd20_transfer_sync(l3gd20_t *gyro, uint8_t *RESTRICT r_buff, uint8_t *RESTRICT w_buff, uint16_t len){
	spi_transfer_t xfer = {
		.read_buff = r_buff,
		.write_buff = w_buff,
		.nss = &gyro->nss,
		.count = len,
		.next = NULL
	};
	spi_transfer(gyro->spi, &xfer);
	while(!xfer.done);
}


