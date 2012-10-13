#ifndef __L3GD20_H_
#define __L3GD20_H_

extern "C" {
	#include "stm32f4xx.h"
}
#include "spi.h"
#include "sensor_config.h"
#include "l3gd20_private.h"


typedef enum {
	L3GD20_ODR_95 = 0,
	L3GD20_ODR_190 = 1,
	L3GD20_ODR_380 = 2,
	L3GD20_ODR_760 = 3
} l3gd20_odr_t;

typedef enum {
	L3GD20_LP_0 = 0,
	L3GD20_LP_1 = 1,
	L3GD20_LP_2 = 2,
	L3GD20_LP_3 = 3
} l3gd20_lp_cutoff_t;

typedef enum {
	L3GD20_HP_OFF = -1,
	L3GD20_HP_0   = 0,
	L3GD20_HP_1   = 1,
	L3GD20_HP_2   = 2,
	L3GD20_HP_3   = 3
} l3gd20_hp_cutoff_t;

/*!
 Platform configuration type for each L3GD20
 @template spiclass The class of the SPI interface
 */
template <typename spiclass>
class l3gd20_config_t {
	float               dps_scale;
	gpio_pin_t          const nss;
	spiclass                 &spi;
};

/*!
 A template class for the L3GD20 to be customized for a platform
 @template spiclass The class of the SPI interface
 */
template <class spiclass> class L3GD20_HW {
public:
	L3GD20_HW(spiclass &_spi, gpio_pin_t const &_nss) :
		spi(_spi), nss(_nss),
		lp_cutoff(L3GD20_LP_3), hp_cutoff(L3GD20_HP_OFF), odr(L3GD20_ODR_95)
			{}
	euclidean3_t        reading;
	
	void set_odr(l3gd20_odr_t odr);
	void set_lp_cutoff(l3gd20_lp_cutoff_t lp_cutoff);
	void set_hp_cutoff(l3gd20_hp_cutoff_t hp_cutoff);
	
	bool init();
	
	void read_sync();
	void read();
	void update();
	
protected:
	void transfer_sync(uint8_t *__restrict r_buff, uint16_t r_len, uint8_t *__restrict w_buff, uint16_t w_len);
	
	l3gd20_odr_t        odr;
	l3gd20_lp_cutoff_t  lp_cutoff;
	l3gd20_hp_cutoff_t  hp_cutoff;
	float               dps_scale;
	gpio_pin_t          const nss;
	spiclass            &spi;
	uint8_t             r_buff[7];
	uint8_t             w_buff[7];
	spi_transfer_t      xfer;
};

template <class spiclass>
bool L3GD20_HW< spiclass >::init(){
	spiclass::slave_init(nss);
	return spi.init();
}

template <class spiclass>
void L3GD20_HW< spiclass >::read_sync(void){
	read();
	//while(!l3gd20_transfer_complete());
	update();
}

template <class spiclass>
void L3GD20_HW< spiclass >::read(){
	int16_t tmp;

	w_buff[0] = GYRO_FLAG_READ | GYRO_FLAG_SEQ | GYRO_REG_OUT_X_L;
	
	xfer.read_buff = r_buff;
	xfer.write_buff = w_buff;
	xfer.nss = &nss;
	xfer.write_count = 1;
	xfer.read_count = 6;
	xfer.next = NULL;

	spi->transfer(&xfer);
}

template <class spiclass>
void L3GD20_HW< spiclass >::update(){
	int16_t tmp;
	tmp = (r_buff[2] << 8) | r_buff[1];
	reading.x = tmp * dps_scale;
	tmp = (r_buff[4] << 8) | r_buff[3];
	reading.y = tmp * dps_scale;
	tmp = (r_buff[6] << 8) | r_buff[5];
	reading.z = tmp * dps_scale;
}

template <class spiclass>
void L3GD20_HW< spiclass >::transfer_sync(uint8_t *__restrict _r_buff, uint16_t _r_len, uint8_t *__restrict _w_buff, uint16_t _w_len){
	spi_transfer_t _xfer;
	decltype(spi)::mk_transfer(_xfer, _r_buff, _w_buff, nss, _w_len, _r_len);
	spi.transfer(&_xfer);
	while(!_xfer.done);
}


void l3gd20_init(void);

/*!
 @brief Start asynchronous sequential reads of all gyroscopes

 Nothing is done upon completion. This function will return very quickly and
 the caller must check l3gd20_transfer_complete to identify when the transfer
 is actually done and then call l3gd20_update to actually update the data
 structures.
 
 @sa l3gd20_transfer_complete()
 @sa l3gd20_update()
 */
void l3gd20_read(void);

/*!
 @brief Update the formatted sensor data fields based on the most recent
 transfer
 */
void l3gd20_update(void);

/*!
 @brief Check to see if the transfer is complete
 @return 0 if incomplete
 */
uint8_t l3gd20_transfer_complete(void);

/*!
 @brief Perform a complete synchronous read of the sensors

 This performs the procedure described in l3gd20_read
 */
void l3gd20_read_sync(void);

#endif

