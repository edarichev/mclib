/**
 * delay.h
 * 
 * Abstraction for time delay and measurement
 */

#ifndef _DELAY_H_INCLUDED_
#define _DELAY_H_INCLUDED_

#if defined(PLATFORM_STM32_HAL)
class Delay
{
public:
	static void wait(uint32_t millisec)
	{
		HAL_Delay(millisec);
	}

	static uint32_t millis()
	{
		return HAL_GetTick();
	}
};
#else
#error "Not supported platform"
#endif

#endif // _DELAY_H_INCLUDED_
