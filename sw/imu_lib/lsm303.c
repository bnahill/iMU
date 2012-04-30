#include "stm32f4xx_conf.h"
#include "i2c.c"
#include "lsm303.h"

#define ACC_ADDR 0b00110010
#define MAG_ADDR 0b00111100

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

void lsm303_init(void){
	uint8_t buffer[32];
	int8_t done;
	buffer[0] = 0;

#if HAS_MAGACC_1
	i2c_init(magacc1.i2c, I2C_MODE_MASTER, 400000);
	done = 0;
	i2c_write(magacc1.i2c, MAG_ADDR, 0x20, buffer, 1, &done);
	while(!done);
#endif
#if HAS_MAGACC_2
	i2c_init(magacc2.i2c, I2C_MODE_MASTER, 400000);
#endif
}


