#include "mma8452q.h"

#define __MMA8452Q_CPP_

#include "mma8452q_platform.h"

void MMA8452Q::init(){
	i2c.init();
	i2c.read_byte(devaddr,REG_WHO_AM_I);
}
