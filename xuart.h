#ifndef _XUART_H_INCLUDED_
#define _XUART_H_INCLUDED_

#include <cstdio>
#include <cstring>
#include <cstdarg>

#if defined(PLATFORM_STM32_HAL)

#if defined(HAL_UART_MODULE_ENABLED)
class XUART
{
	UART_HandleTypeDef *_huart = nullptr;
public:
	XUART(UART_HandleTypeDef *huart) : _huart(huart)
	{

	}

	void write(const char *format, ...)
	{
		char msg[80];
		va_list args;
		va_start(args, format);
		vsprintf(msg, format, args);
		va_end(args);
		_write(msg);
	}

	void writeLine(const char *format, ...)
	{
		char msg[80];
		va_list args;
		va_start(args, format);
		vsprintf(msg, format, args);
		va_end(args);
		_writeLine(msg);
	}
private:
	void _write(const char *msg)
	{
		HAL_UART_Transmit(_huart, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}

	void _writeLine(const char *msg)
	{
		_write(msg);
		const char *EOL = "\r\n";
		HAL_UART_Transmit(_huart, (uint8_t*) EOL, strlen(EOL), HAL_MAX_DELAY);
	}
};
#endif // defined(HAL_UART_MODULE_ENABLED)

#else
#endif

#endif // _XUART_H_INCLUDED_
