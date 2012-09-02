#include "sensor_config.h"

#if USE_L3GD20

#include "stm32f4xx_conf.h"

#include "spi.h"
#include "l3gd20.h"
#include "l3gd20_private.h"


//! A buffer for the command to read values from a gyroscope
static uint8_t gyro_write_buffer[7];

//! @name Private
//! @{
static uint8_t l3gd20_read_register(l3gd20_t *__restrict gyro, uint8_t addr);
static void l3gd20_configure_device(l3gd20_t *__restrict gyro);
static void l3gd20_device_init(l3gd20_t *gyro);
static void l3gd20_write_register(l3gd20_t *__restrict gyro, uint8_t addr, uint8_t value);
static void l3gd20_transfer_sync(l3gd20_t *gyro, uint8_t *__restrict r_buff, uint16_t r_len, uint8_t *__restrict w_buff, uint16_t w_len);
static void l3gd20_read_sensor(l3gd20_t *__restrict gyro);
static void l3gd20_update_device(l3gd20_t * const gyro);
static volatile int is_done;
// @}

//////////////////////////////////////////////////////////////////////////////
// Public functions
//////////////////////////////////////////////////////////////////////////////



void l3gd20_read_sync(void){
	l3gd20_read();
	while(!l3gd20_transfer_complete());
	l3gd20_update();
}


//////////////////////////////////////////////////////////////////////////////
// Private support functions
//////////////////////////////////////////////////////////////////////////////
static void test();

static void l3gd20_reset(l3gd20_t *__restrict gyro){
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG5, 0x80);
}

static void l3gd20_configure_device(l3gd20_t *__restrict gyro){
	uint8_t read, write;
	
	l3gd20_reset(gyro);
	
	// Enable BDU (delay 1 cycle), little endian, 500dps, 4-wire SPI
	write = 0b10010000;
	// Configure for 3-wire SPI if needed
	if(gyro->spi->config->mode == SPI_MODE_3WIRE)
		write |= 0x01; 
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG4, write);
	read = l3gd20_read_register(gyro, GYRO_REG_CTRL_REG4);
	if(read != write)
		while(1);
	
	//test();

	read = l3gd20_read_register(gyro, GYRO_REG_WHO_AM_I);
	if(read != 0xD4)
		while(1);
	
	write = 0b01111111;
	// Set for 190Hz with 70Hz BW, normal mode with all axes on
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG1, write);
	if(l3gd20_read_register(gyro, GYRO_REG_CTRL_REG1) != write)
		while(1);

	// Set 0.018Hz HP
	write = 0b00001001;
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG2, write);
	read = l3gd20_read_register(gyro, GYRO_REG_CTRL_REG2);
	if(read != write)
		while(1);
	
	// Disable all interrupts
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG3, 0x00);
	
	// Enable HP filter and use it
	write = 0b00010001;
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG5, write);
	write = 0b00010001;
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG5, write);
	write = 0b00010001;
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG5, write);
	write = 0b00010001;
	l3gd20_write_register(gyro, GYRO_REG_CTRL_REG5, write);

	/*
	read = l3gd20_read_register(gyro, GYRO_REG_CTRL_REG5);
	if(read != write)
		while(1);
	*/
	// FIFO bypass mode
	l3gd20_write_register(gyro, GYRO_REG_FIFO_CTRL, 0x00);
}

static void l3gd20_device_init(l3gd20_t *gyro){
	spi_init(gyro->spi);
	l3gd20_configure_device(gyro);
}


static void l3gd20_write_register(l3gd20_t *__restrict gyro, uint8_t addr, uint8_t value){
	uint8_t read_buff[2], write_buff[2];
	write_buff[0] = GYRO_FLAG_WRITE | GYRO_FLAG_NOSEQ | addr;
	write_buff[1] = value;
	l3gd20_transfer_sync(gyro, read_buff, 0, write_buff, 2);
}

static uint8_t l3gd20_read_register(l3gd20_t *__restrict gyro, uint8_t addr){
	uint8_t read_buff[3], write_buff[2];
	write_buff[0] = GYRO_FLAG_READ | GYRO_FLAG_NOSEQ | addr;
	write_buff[1] = 0;
	read_buff[0] = 0;
	read_buff[1] = 0;
	read_buff[2] = 0;
	l3gd20_transfer_sync(gyro, read_buff, 1, write_buff, 1);
	return read_buff[1];
}

static void l3gd20_read_sensor(l3gd20_t *__restrict gyro){
	int16_t tmp;

	gyro_write_buffer[0] = GYRO_FLAG_READ | GYRO_FLAG_SEQ | GYRO_REG_OUT_X_L;
	
	gyro->xfer.read_buff = gyro->r_buff;
	gyro->xfer.write_buff = gyro_write_buffer;
	gyro->xfer.nss = &gyro->nss;
	gyro->xfer.write_count = 1;
	gyro->xfer.read_count = 6;
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

static void l3gd20_transfer_sync(l3gd20_t *gyro, uint8_t *__restrict r_buff, uint16_t r_len, uint8_t *__restrict w_buff, uint16_t w_len){
	spi_transfer_t xfer = {
		.read_buff = r_buff,
		.write_buff = w_buff,
		.nss = &gyro->nss,
		.write_count = w_len,
		.read_count = r_len,
		.next = NULL
	};
	spi_transfer(gyro->spi, &xfer);
	while(!xfer.done);
}

// Include the platform-specific components
#define _L3GD20_C__
#include "l3gd20_platform.h"

static void test(){
	uint8_t rb[66], wb = 0xC0;
	spi_transfer_t xfer = {
		.next = NULL,
		.read_buff = rb,
		.write_buff = &wb,
		.read_count = 64,
		.write_count = 1,
		.nss = &gyro1.nss
	};
	spi_transfer(gyro1.spi, &xfer);
	while(!xfer.done);
}

#endif // USE_L3GD20

