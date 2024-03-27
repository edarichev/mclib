#ifndef _XUART_H_INCLUDED_
#define _XUART_H_INCLUDED_

#include <cstdio>
#include <cstring>

#if defined(PLATFORM_STM32_HAL)

#if defined(HAL_UART_MODULE_ENABLED)
class XUART
{
	UART_HandleTypeDef *_huart = nullptr;
public:
	XUART(UART_HandleTypeDef *huart) : _huart(huart)
	{

	}

	void write(const char *msg)
	{
		HAL_UART_Transmit(_huart, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}
	void writeLine(const char *msg)
	{
		HAL_UART_Transmit(_huart, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		const char *EOL = "\r\n";
		HAL_UART_Transmit(_huart, (uint8_t*) EOL, strlen(EOL), HAL_MAX_DELAY);
	}

	void write(const char *format, uint32_t value)
	{
		char msg[80];
		sprintf(msg, format, value);
		write(msg);
	}

	void write(const char *format, float value)
	{
		char msg[80];
		sprintf(msg, format, value);
		write(msg);
	}

	void writeLine(const char *format, uint32_t value)
	{
		char msg[80];
		sprintf(msg, format, value);
		writeLine(msg);
	}
};
#endif // defined(HAL_UART_MODULE_ENABLED)

#else
#endif

#endif // _XUART_H_INCLUDED_
