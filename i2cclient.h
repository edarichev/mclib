/**
 * i2cclient.h
 * 
 * I2C Abstraction
 */

#ifndef _I2CCLIENT_H_INCLUDED_
#define _I2CCLIENT_H_INCLUDED_

// Platform specific realization

#if defined(PLATFORM_STM32_HAL)
class I2C
{
private:
	I2C_HandleTypeDef *_hi2c = nullptr;
public:
	I2C(I2C_HandleTypeDef *hi2cInstance) :
			_hi2c(hi2cInstance)
	{
	}

	bool write(uint16_t addr, uint8_t b, uint32_t timeout) const
	{
		return HAL_OK == HAL_I2C_Master_Transmit(_hi2c, addr, &b, 1, timeout);
	}

	bool read(uint16_t addr, uint8_t *pBuffer, uint16_t size,
			uint32_t timeout) const
	{
		return HAL_OK == HAL_I2C_Master_Receive(_hi2c, addr, pBuffer, size, timeout);
	}
};
#else
#error "Not supported platform"
#endif

class I2CClient
{
protected:
	I2C *_i2c = nullptr;

	// Public fields. This instance may be used as Flyweight Pattern.
public:
	uint16_t I2CAddress = 0x00;
	uint32_t Timeout = 300;
protected:
	I2CClient(I2C *i2c, uint16_t addr) :
			_i2c(i2c), I2CAddress(addr)
	{
	}

	inline bool write(uint8_t b) const
	{
		return _i2c->write(I2CAddress, b, Timeout);
	}

	inline bool read(uint8_t *pBuffer, uint16_t size) const
	{
		return _i2c->read(I2CAddress, pBuffer, size, Timeout);
	}
};



#endif // _I2CCLIENT_H_INCLUDED_
