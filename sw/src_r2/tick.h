#ifndef __TICK_H_
#define __TICK_H_

extern "C"{
	#include "stm32f4xx.h"
}

//! @addtogroup tick Tick
//! @{
class Tick {
public:
	/*!
	@brief Start the timer with specified period
	@param period_ms Number of milliseconds in period
	@return True if successful
	*/
	static int start(float period_ms);

	/*!
	@brief Wait for some number of ticks
	@param num_ticks Number of ticks to wait
	*/
	static void wait(uint32_t num_ticks);

	
	static volatile int tick;
};
//! @}

#endif

