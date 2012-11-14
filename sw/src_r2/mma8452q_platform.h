#ifndef __MMA8452Q_PLATFORM_H_
#define __MMA8452Q_PLATFORM_H_

#include "mma8452q.h"
#include "i2c_platform.h"

#ifdef __MMA8452Q_CPP_
MMA8452Q acc1(i2c1, 0x1C);
#endif

extern MMA8452Q acc1;

#endif
