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
	uint32_t Timeout = 1000;
public:
	UARTClient(UART_HandleTypeDef *huart) : _huart(huart) {}

	bool read(uint8_t *buf, uint16_t bufSize)
	{
	    return _read(buf, bufSize);
	}

	/**
	 * Writes the string to UART. Uses buffer on the stack with 80 chars.
	 */
	bool write(const char *format, ...)
	{
		char msg[80];
		va_list args;
		va_start(args, format);
		vsprintf(msg, format, args);
		va_end(args);
		return _write(msg);
	}

	/**
	 * Writes the string to UART. Uses dinamycally allocated buffer with specified size.
	 */
	bool write(size_t bufAllocSize, const char *format, ...)
	{
		if (!bufAllocSize)
			return false;
		char *msg = new char [bufAllocSize];
		va_list args;
		va_start(args, format);
		vsprintf(msg, format, args);
		va_end(args);
		bool ok = _write(msg);
		delete [] msg;
		return ok;
	}

	/**
	 * Writes the string with \r\n ending to UART.
	 * Uses buffer on the stack with 80 chars.
	 */
	bool writeLine(const char *format, ...)
	{
		char msg[80];
		va_list args;
		va_start(args, format);
		vsprintf(msg, format, args);
		va_end(args);
		return _writeLine(msg);
	}

	bool writeLine(size_t bufAllocSize, const char *format, ...)
	{
		if (!bufAllocSize)
			return false;
		char *msg = new char [bufAllocSize];
		va_list args;
		va_start(args, format);
		vsprintf(msg, format, args);
		va_end(args);
		bool ok = _writeLine(msg);
		delete [] msg;
		return ok;
	}
protected:

	bool _read(uint8_t *buf, uint16_t bufSize)
	{
	    return HAL_OK == HAL_UART_Receive(_huart, buf, bufSize, Timeout);
	}

	bool _write(const char *msg)
	{
		return HAL_OK == HAL_UART_Transmit(_huart, (uint8_t*) msg, strlen(msg), Timeout);
	}

	bool _writeLine(const char *msg)
	{
		if (!_write(msg))
		    return false;
		const char *EOL = "\r\n";
		return HAL_OK == HAL_UART_Transmit(_huart, (uint8_t*) EOL, strlen(EOL), Timeout);
	}
};

class UARTLogger : public UARTClient
{
public:
	UARTLogger(UART_HandleTypeDef *huart) : UARTClient(huart)
	{

	}

	// TODO: create debug & release version?
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

#else
#error "Enable at least one UART interface"
#endif // defined(HAL_UART_MODULE_ENABLED)

#else
#endif

#endif // _XUART_H_INCLUDED_
