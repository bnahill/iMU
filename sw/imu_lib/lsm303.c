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

#if HAS_MAGACC_1
lsm303_t magacc1 = {
	{0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0},
	&i2c1
};
#endif

#if HAS_MAGACC_2
lsm303_t magacc2 = {
	{0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0},
	&i2c2
};
#endif

void lsm303_set_odr(lsm303_t *lsm, lsm_rate_t rate){
	i2c_write_byte(lsm->i2c, ACC_ADDR, ACC_REG_CTRL1, rate << 4);
}

static void lsm303_device_init(lsm303_t *lsm){
	// Initialize I2C device
	i2c_init(lsm->i2c, I2C_MODE_MASTER, 400000);

	lsm303_set_odr(lsm, LSM_RATE_100);
}

void lsm303_init(void){
#if HAS_MAGACC_1
	lsm303_device_init(&magacc1);
#endif
#if HAS_MAGACC_2
	lsm303_device_init(&magacc2);
#endif
}


