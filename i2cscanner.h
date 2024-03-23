/**
 * I2C Scanner: enumerate I2C devices
 * 22.03.2024 21:30:16
 * Author: de
 */

#ifndef __I2CSCANNER_H_INCLUDED__
#define __I2CSCANNER_H_INCLUDED__

#include <functional>

#if defined(PLATFORM_STM32_HAL)

#if defined(HAL_I2C_MODULE_ENABLED)

class I2CScanner
{
public:
	static uint8_t enumerate(I2C_HandleTypeDef *hi2c, std::function<void(uint8_t foundAddress)> foundCallback)
	{
		const uint32_t trials = 3; // try 3 times
		const uint32_t timeout = 1000; // 1 sec
		uint8_t count = 0;
		for (uint8_t i = 1; i < 127; ++i) {
			HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(i<<1), trials, timeout);
			if(ret == HAL_OK) // ACK received
			{
				foundCallback(i);
				count++;
			}
		}
		return count;
	}
};

#endif // HAL_I2C_MODULE_ENABLED

#else
#endif
#endif // __I2CSCANNER_H_INCLUDED__

/* Example
// enumerate I2C devices:
  uint8_t ic = I2CScanner::enumerate(&hi2c1, [](auto addr) {
	  uartmsgln("I2C device found at address %#2X", addr);
  });
  uartmsgln("%d I2C devices found", ic);
 *
 */
