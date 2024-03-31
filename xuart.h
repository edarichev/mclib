#ifndef _XUART_H_INCLUDED_
#define _XUART_H_INCLUDED_

#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <memory>
#include <stdlib.h>

#if defined(PLATFORM_STM32_HAL)

#if defined(HAL_UART_MODULE_ENABLED)

class UARTClient
{
private:
	UART_HandleTypeDef *_huart = nullptr;
public:
	UARTClient(UART_HandleTypeDef *huart) : _huart(huart) {}

	/**
	 * Writes the string to UART. Uses buffer on the stack with 80 chars.
	 */
	void write(const char *format, ...)
	{
		char msg[80];
		va_list args;
		va_start(args, format);
		vsprintf(msg, format, args);
		va_end(args);
		_write(msg);
	}

	/**
	 * Writes the string to UART. Uses dinamycally allocated buffer with specified size.
	 */
	void write(size_t bufAllocSize, const char *format, ...)
	{
		if (!bufAllocSize)
			return;
		char *msg = new char [bufAllocSize];
		va_list args;
		va_start(args, format);
		vsprintf(msg, format, args);
		va_end(args);
		_write(msg);
		delete [] msg;
	}

	/**
	 * Writes the string with \r\n ending to UART.
	 * Uses buffer on the stack with 80 chars.
	 */
	void writeLine(const char *format, ...)
	{
		char msg[80];
		va_list args;
		va_start(args, format);
		vsprintf(msg, format, args);
		va_end(args);
		_writeLine(msg);
	}

	void writeLine(size_t bufAllocSize, const char *format, ...)
	{
		if (!bufAllocSize)
			return;
		char *msg = new char [bufAllocSize];
		va_list args;
		va_start(args, format);
		vsprintf(msg, format, args);
		va_end(args);
		_writeLine(msg);
		delete [] msg;
	}
protected:
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

class UARTLogger : public UARTClient
{
public:
	UARTLogger(UART_HandleTypeDef *huart) : UARTClient(huart)
	{

	}

	// TODO: create debug & release version
	void xassert(bool condition, const char *format, ...)
	{
		if (condition)
			return;
        va_list args;
        va_start(args, format);
		writeLine(format, args);
        va_end(args);
	}
};

#endif // defined(HAL_UART_MODULE_ENABLED)

#else
#endif

#endif // _XUART_H_INCLUDED_
