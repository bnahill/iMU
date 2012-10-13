#include "tick.h"

//! A single millisecond tick
#define SYSTICK_MS(ms) ((ms*SystemCoreClock) / 8000.0)

//! The actual tick resource
volatile int Tick::tick;

int Tick::start(float period_ms){
	if(SysTick_Config(SYSTICK_MS(period_ms)))
		return 0;
	tick = 0;
	// THEN specify to divide this clock by 8
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	return 1;
}

void Tick::wait(uint32_t num_ticks){
	while(num_ticks--){
		while(tick);
		tick = 0;
	}
}

extern "C"{
	void SysTick_Handler(void);
}

void SysTick_Handler(void){
	Tick::tick = 1;
}
