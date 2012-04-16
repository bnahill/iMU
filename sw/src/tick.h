#ifndef __TICK_H_
#define __TICK_H_

#include "stm32f4xx_conf.h"

//! @addtogroup tick Tick
//! @{

/*!
 @brief Start the timer with specified period
 @param period_ms Number of milliseconds in period
 @return True if successful
 */
int tick_start(float period_ms);

/*!
 @brief Wait for some number of ticks
 @param num_ticks Number of ticks to wait
 */
void tick_wait(uint32_t num_ticks);

//! @}

#endif

