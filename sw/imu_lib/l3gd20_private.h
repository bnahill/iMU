#ifndef _L3GD20_PRIVATE_H__
#define _L3GD20_PRIVATE_H__

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

#endif
