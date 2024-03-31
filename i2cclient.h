/**
 * i2cclient.h
 * 
 * I2C Abstraction
 */

#ifndef _I2CCLIENT_H_INCLUDED_
#define _I2CCLIENT_H_INCLUDED_

#include <type_traits>

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

enum I2CConnectionMode : uint8_t
{
	Polling,
	DMA,
	Interrupt
};

// Platform specific realization


#if defined(PLATFORM_STM32_HAL) && defined(HAL_I2C_MODULE_ENABLED)

// Before including this file include the your platform-specific file,
// for example "stm32f4xx_hal.h" for STM32 F4 board

class I2C
{
protected:
	I2C_HandleTypeDef *_hi2c = nullptr;
protected:
	I2C(I2C_HandleTypeDef *hi2cInstance) :
			_hi2c(hi2cInstance) {}
};

class I2CPollingModeMaster : public I2C
{
public:
	I2CPollingModeMaster(I2C_HandleTypeDef *hi2cInstance) :
			I2C(hi2cInstance)
	{
	}

	inline bool write(uint16_t addr, uint8_t b, uint32_t timeout) const
	{
		return write(addr, &b, 1, timeout);
	}

	inline bool write(uint16_t addr, uint8_t *b, uint16_t size, uint32_t timeout) const
	{
		return HAL_OK == HAL_I2C_Master_Transmit(_hi2c, addr, b, size, timeout);
	}

	bool read(uint16_t addr, uint8_t *pBuffer, uint16_t size,
			uint32_t timeout) const
	{
		return HAL_OK == HAL_I2C_Master_Receive(_hi2c, addr, pBuffer, size, timeout);
	}

	bool memRead(uint16_t addr, uint16_t memAddr, uint8_t *buf, uint16_t size,
			uint32_t timeout) const
	{
		// TODO: The value I2C_MEMADD_SIZE_8BIT may be wrong for some cases, check it
		return HAL_OK == HAL_I2C_Mem_Read(_hi2c, addr, memAddr,
				I2C_MEMADD_SIZE_8BIT, buf, size, timeout);
	}

	bool memWrite(uint16_t addr, uint16_t memAddr, uint8_t *buf, uint16_t size,
			uint32_t timeout) const
	{
		// TODO: The value I2C_MEMADD_SIZE_8BIT may be wrong for some cases, check it
		return HAL_OK == HAL_I2C_Mem_Write(_hi2c, addr, memAddr,
				I2C_MEMADD_SIZE_8BIT, buf, size, timeout);
	}
};

#else
#error "Not supported platform"
#endif

// Base class for type checking
class I2CClientBase
{
	// Public fields.
public:
	uint16_t I2CAddress = 0x00;
	uint32_t Timeout = 300;
protected:
	I2CClientBase(uint16_t addr) : I2CAddress(addr) {}
};

template <class TI2C>
class I2CClientImpl : public I2CClientBase
{
protected:
	TI2C *_i2c = nullptr;
public:
	using InterfaceType = TI2C;
	using InterfacePtrType = TI2C*;
protected:
	I2CClientImpl(TI2C *i2c, uint16_t addr) :
		I2CClientBase(addr), _i2c(i2c)
	{
	}

	inline bool write(uint8_t b) const
	{
		return _i2c->write(I2CAddress, b, Timeout);
	}

	inline bool write(uint8_t *b, uint16_t size) const
	{
		return _i2c->write(I2CAddress, b, size, Timeout);
	}

	inline bool read(uint8_t *pBuffer, uint16_t size) const
	{
		return _i2c->read(I2CAddress, pBuffer, size, Timeout);
	}

	inline bool read(uint8_t *b) const
	{
		return _i2c->read(I2CAddress, b, 1, Timeout);
	}

	inline bool memRead(uint16_t memAddr, uint8_t *buf, uint16_t size) const
	{
		return _i2c->memRead(I2CAddress, memAddr, buf, size, Timeout);
	}

	inline bool memWrite(uint16_t memAddr, uint8_t *buf, uint16_t size) const
	{
		return _i2c->memWrite(I2CAddress, memAddr, buf, size, Timeout);
	}
};

using I2CPollingClient = I2CClientImpl<I2CPollingModeMaster>;

#endif // _I2CCLIENT_H_INCLUDED_
