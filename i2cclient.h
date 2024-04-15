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

	inline bool isDeviceReady(uint16_t addr, uint32_t trials, uint32_t timeout) const
	{
	    return HAL_OK == HAL_I2C_IsDeviceReady(_hi2c, addr, trials, timeout);
	}

	inline bool write(uint16_t addr, uint8_t b, uint32_t timeout) const
	{
		return write(addr, &b, 1, timeout);
	}

	inline bool write(uint16_t addr, const uint8_t *b, uint16_t size, uint32_t timeout) const
	{
		return HAL_OK == HAL_I2C_Master_Transmit(_hi2c, addr, (uint8_t*)b, size, timeout);
	}

	bool read(uint16_t addr, uint8_t *pBuffer, uint16_t size,
			uint32_t timeout) const
	{
		return HAL_OK == HAL_I2C_Master_Receive(_hi2c, addr, pBuffer, size, timeout);
	}

	bool memRead(uint16_t addr, uint16_t memAddr, uint16_t addrSize, uint8_t *buf, uint16_t size,
	        uint32_t timeout) const
	{

        switch (addrSize) {
        case 16:
            addrSize = I2C_MEMADD_SIZE_16BIT;
            break;
        case 8:
        default:
            addrSize = I2C_MEMADD_SIZE_8BIT;
            break;
        }
		return HAL_OK == HAL_I2C_Mem_Read(_hi2c, addr, memAddr,
		        addrSize, buf, size, timeout);
	}

	bool memWrite(uint16_t addr, uint16_t memAddr, uint16_t addrSize, const uint8_t *buf, uint16_t size,
	        uint32_t timeout) const
	{
	    uint16_t halAddMemSize = I2C_MEMADD_SIZE_16BIT;
	    switch (addrSize) {
	    case 16:
	        halAddMemSize = I2C_MEMADD_SIZE_16BIT;
	        break;
	    case 8:
	    default:
	        halAddMemSize = I2C_MEMADD_SIZE_8BIT;
	        break;
	    }
		return HAL_OK == HAL_I2C_Mem_Write(_hi2c, addr, memAddr,
		        halAddMemSize, (uint8_t*)buf, size, timeout);
	}
#if 0
// Alternative:
    /**
     * Reads from memory via I2C
     *
     * @param addSize address size in bits: 8 or 16
     */
    bool memoryRead(uint16_t memAddr, uint8_t *pBuf, uint16_t bufSize,
            uint8_t addrSize = 16)
    {
        uint8_t addr[2];
        size_t addressSize = 2;
        if (addrSize == 16) {
            addr[0] = (uint8_t)(memAddr >> 8); // MSB of memory address
            addr[1] = (uint8_t)(memAddr & 0xFF); // LSB of memory address
        } else {
            addr[0] = (uint8_t)(memAddr & 0xFF);
            addressSize = 1;
        }
        // send address of starting memory address to read
        if (!TInterfaceClient::write(addr, addressSize))
            return false;
        if (!TInterfaceClient::read(pBuf, bufsize))
            return false;
        return true;
    }
#endif
};

#else
#error "Not supported platform"
#endif

// Base class for type checking
class I2CClientBase
{
	// Public fields.

public:
    // I2C address.
    // You can modify address directly, for example, if you have multiple EEPROM's,
    // you can change this address and work (i.e. it can be used as the Flyweight Pattern).
	uint16_t I2CAddress = 0x0000;
	uint32_t Timeout = 1000;
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

	inline bool write(const uint8_t *b, uint16_t size) const
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

	inline bool memRead(uint16_t memAddr, uint16_t addrSize, uint8_t *buf, uint16_t size) const
	{
		return _i2c->memRead(I2CAddress, memAddr, addrSize, buf, size, Timeout);
	}

	inline bool memWrite(uint16_t memAddr, uint16_t addrSize, const uint8_t *buf, uint16_t size) const
	{
		return _i2c->memWrite(I2CAddress, memAddr, addrSize, buf, size, Timeout);
	}

	inline bool isDeviceReady(uint32_t trials = 1) const
    {
        return _i2c->isDeviceReady(I2CAddress, trials, Timeout);
    }
};

using I2CPollingClient = I2CClientImpl<I2CPollingModeMaster>;

#endif // _I2CCLIENT_H_INCLUDED_
