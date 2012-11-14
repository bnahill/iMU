#ifndef __MMA8452Q_H_
#define __MMA8452Q_H_

#include "sensor_config.h"
#include "i2c_platform.h"

class MMA8452Q {
public:
	//! Define the type of I2C that can be accepted. This device will only
	//! work with 7-bit device addressing and 8-bit registers
	typedef I2CMaster_7bDev_8bRegaddrs i2c_t;
	
	MMA8452Q(i2c_t &i2c, uint8_t devaddr) : 
		i2c(i2c), devaddr(devaddr)
		{}

	void init();
		
private:
	typedef enum {
		REG_STATUS        = 0x00,
		REG_OUT_X_MSB     = 0x01,
		REG_OUT_X_LSB     = 0x02,
		REG_OUT_Y_MSB     = 0x03,
		REG_OUT_Y_LSB     = 0x04,
		REG_OUT_Z_MSB     = 0x05,
		REG_OUT_Z_LSB     = 0x06,
		REG_SYSMOD        = 0x0B,
		REG_INT_SOURCE    = 0x0C,
		REG_WHO_AM_I      = 0x0D,
		REG_XYZ_DATA_CFG  = 0x0E,
		REG_HP_CUTOFF     = 0x0F,
		REG_PL_STATUS     = 0x10,
		REG_PL_CFG        = 0x11,
		REG_PL_COUNT      = 0x12,
		REG_PL_BF_ZCOMP   = 0x13,
		REG_PL_THS_REG    = 0x14,
		REG_FF_MT_CFG     = 0x15,
		REG_FF_MT_SRC     = 0x16,
		REG_FF_MT_THS     = 0x17,
		REG_FF_MT_COUNT   = 0x18,
		REG_TRANS_CFG     = 0x1E,
		REG_TRANS_SRC     = 0x1F,
		REG_TRANS_THS     = 0x20,
		REG_TRANS_COUNT   = 0x21,
		REG_PULSE_CFG     = 0x22,
		REG_PULSE_SRC     = 0x23,
		REG_PULSE_THSX    = 0x24,
		REG_PULSE_THSY    = 0x25,
		REG_PULSE_THSZ    = 0x26,
		REG_PULSE_TMLT    = 0x27,
		REG_PULSE_LTCY    = 0x28,
		REG_PULSE_WIND    = 0x29,
		REG_ASLP_COUNT    = 0x2A,
		REG_CTRL_REG1     = 0x2B,
		REG_CTRL_REG2     = 0x2C,
		REG_CTRL_REG3     = 0x2D,
		REG_CTRL_REG4     = 0x2E,
		REG_CTRL_REG5     = 0x2F,
		REG_OFF_X         = 0x30,
		REG_OFF_Y         = 0x31,
		REG_OFF_Z         = 0x0D
	} reg_t;
	
	uint8_t devaddr;
	
	//! A transfer to use for periodic reads
	i2c_t::xfer_t xfer;
	
	//! I2C device to use
	i2c_t &i2c;
};

#endif
