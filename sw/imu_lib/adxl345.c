#include "sensor_config.h"
#if USE_ADXL345

#include "adxl345.h"

void adxl345_init(adxl345_t *__restrict acc){
	spi_init_slave(&acc->nss);
}

// Include the platform-specific components
#define _ADXL345_C__
#include "adxl345_platform.h"

#endif // USE_ADXL345