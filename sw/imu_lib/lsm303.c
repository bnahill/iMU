#include "stm32f4xx_conf.h"
#include "i2c.c"
#include "lsm303.h"

//! @name I2C Addresses
//! @{
#define ACC_ADDR 0x32
#define MAG_ADDR 0x3C
//! @}

//! @name Accelerometer register addresses
//! @{
#define ACC_REG_CTRL1     0x20
#define ACC_REG_CTRL2     0x21
#define ACC_REG_CTRL3     0x22
#define ACC_REG_CTRL4     0x23
#define ACC_REG_CTRL5     0x24
#define ACC_REG_CTRL6     0x25
#define ACC_REG_REFA      0x26
#define ACC_REG_STAT      0x27
#define ACC_REG_X_L       0x28
#define ACC_REG_X_H       0x29
#define ACC_REG_Y_L       0x2A
#define ACC_REG_Y_H       0x2B
#define ACC_REG_Z_L       0x2C
#define ACC_REG_Z_H       0x2D
#define ACC_REG_FIFO_CTL  0x2E
#define ACC_REG_FIFO_SRC  0x2F
#define ACC_REG_INT1_CFG  0x30
#define ACC_REG_INT1_SRC  0x31
#define ACC_REG_INT1_THS  0x32
#define ACC_REG_INT1_DUR  0x33
#define ACC_REG_INT2_CFG  0x34
#define ACC_REG_INT2_SRC  0x35
#define ACC_REG_INT2_THS  0x36
#define ACC_REG_INT2_DUR  0x37
#define ACC_REG_CLCK_CFG  0x38
#define ACC_REG_CLCK_SRC  0x39
#define ACC_REG_CLCK_THS  0x3A
#define ACC_REG_TIME_LIM  0x3B
#define ACC_REG_TIME_LAT  0x3C
#define ACC_REG_TIME_WIN  0x3D
//! }@

//! @name Magnetometer register addresses
//! @{
#define MAG_REG_CRA       0x00
#define MAG_REG_CRB       0x01
#define MAG_REG_MR        0x02
#define MAG_REG_X_H       0x03
#define MAG_REG_X_L       0x04
#define MAG_REG_Z_H       0x05
#define MAG_REG_Z_L       0x06
#define MAG_REG_Y_H       0x07
#define MAG_REG_Y_L       0x08
#define MAG_REG_SR        0x09
#define MAG_REG_IRA       0x0A
#define MAG_REG_IRB       0x0B
#define MAG_REG_IRC       0x0C
#define MAG_REG_TEMP_H    0x31
#define MAG_REG_TEMP_L    0x32
//! @}

const float acc_scale[] = {
	4.0 / 4096.0,
	8.0 / 4096.0,
	16.0 / 4096.0,
	32.0 / 4096.0
};

const float mag_scale[] = {
	0,
	1.0 / 1100.0,
	1.0 / 855.0,
	1.0 / 670.0,
	1.0 / 450.0,
	1.0 / 400.0,
	1.0 / 330.0,
	1.0 / 230.0
};

#if HAS_MAGACC_1
lsm303_t magacc1 = {
	{0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0},
	&i2c1,
	.rate = LSM_RATE_100,
	.acc_fs = LSM_ACC_FS_4G,
	.mag_fs = LSM_MAG_FS_1_3
};
#endif

#if HAS_MAGACC_2
lsm303_t magacc2 = {
	{0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0},
	&i2c2,
	.rate = LSM_RATE_100,
	.acc_fs = LSM_ACC_FS_4G,
	.mag_fs = LSM_MAG_FS_1_3
};
#endif

void lsm303_set_acc_fs(lsm303_t *lsm, lsm_acc_fs_t fs){
	uint8_t current = i2c_read_byte(lsm->i2c, ACC_ADDR, ACC_REG_CTRL4 | 0x80) & 0xCF;
	current |= (fs & 0x3) << 4;
	i2c_write_byte(lsm->i2c, ACC_ADDR, ACC_REG_CTRL4, current);
	lsm->acc_fs = fs;
}

void lsm303_set_rate(lsm303_t *lsm, lsm_rate_t rate){
	lsm->rate = rate;
	i2c_write_byte(lsm->i2c, ACC_ADDR, ACC_REG_CTRL1, rate << 4);
}

static void lsm303_device_init(lsm303_t *lsm){
	// Init CTRL1-6
	uint8_t conf_buffer[6] = {
		(lsm->rate << 4) | 0x07,
		0,
		0,
		0x80 | (lsm->acc_fs << 4),
		0,
		0
	};

	i2c_transfer_t xfer = {
		.op = I2C_OP_WRITE,
		.devaddr = ACC_ADDR,
		.addr = ACC_REG_CTRL1 | 0x80,
		.buffer = conf_buffer,
		.count = 6,
		.next = NULL
	};

	// Initialize I2C device
	i2c_init(lsm->i2c, I2C_MODE_MASTER, 400000);

	// Apply accelerometer configuration
	i2c_transfer(lsm->i2c, &xfer);
	while(!xfer.done);

	xfer.devaddr = MAG_ADDR;
	xfer.addr = MAG_REG_CRA | 0x80;
	xfer.buffer = conf_buffer;
	conf_buffer[0] = 0x98;
	conf_buffer[1] = lsm->mag_fs << 5;
	conf_buffer[2] = 0;
	xfer.count = 3;

	// Apply magnetometer configuration
	i2c_transfer(lsm->i2c, &xfer);
	while(!xfer.done);
}

static void lsm303_do_read(lsm303_t *lsm){
	uint8_t acc_buff[6], mag_buff[6];
	float tmp;
	int16_t tmp16;
	i2c_transfer_t mag_xfer, acc_xfer;

	mag_xfer.op = I2C_OP_READ;
	mag_xfer.devaddr = MAG_ADDR;
	mag_xfer.addr = MAG_REG_X_H | 0x80;
	mag_xfer.buffer = mag_buff;
	mag_xfer.count = 6;
	mag_xfer.next = NULL;
	
	acc_xfer.op = I2C_OP_READ;
	acc_xfer.devaddr = ACC_ADDR;
	acc_xfer.addr = ACC_REG_X_L | 0x80;
	acc_xfer.buffer = acc_buff;
	acc_xfer.count = 6;
	acc_xfer.next = &mag_xfer;
	
	i2c_transfer(lsm->i2c, &acc_xfer);
	
	while(!acc_xfer.done);
	tmp16 = acc_buff[0] | (acc_buff[1] << 8);
	lsm->acc.x = (tmp16 >> 4) * acc_scale[lsm->acc_fs];
	tmp16 = acc_buff[2] | (acc_buff[3] << 8);
	lsm->acc.y = (tmp16 >> 4) * acc_scale[lsm->acc_fs];
	tmp16 = acc_buff[4] | (acc_buff[5] << 8);
	lsm->acc.z = (tmp16 >> 4) * acc_scale[lsm->acc_fs];
	while(!mag_xfer.done);
	tmp16 = mag_buff[1] | (mag_buff[0] << 8);
	lsm->mag.x = tmp16 * mag_scale[lsm->mag_fs];
	tmp16 = mag_buff[3] | (mag_buff[2] << 8);
	lsm->mag.z = tmp16 * mag_scale[lsm->mag_fs];
	tmp16 = mag_buff[5] | (mag_buff[4] << 8);
	lsm->mag.y = tmp16 * mag_scale[lsm->mag_fs];
}

void lsm303_init(void){
#if HAS_MAGACC_1
	lsm303_device_init(&magacc1);
#endif
#if HAS_MAGACC_2
	lsm303_device_init(&magacc2);
#endif
}

void lsm303_read(void){
#if HAS_MAGACC_1
	lsm303_do_read(&magacc1);
#endif
#if HAS_MAGACC_2
	lsm303_do_read(&magacc2);
#endif
}


