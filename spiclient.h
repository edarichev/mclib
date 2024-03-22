/**
 * spiclient.h
 * 
 * I2C Abstraction

 *  Created on: Mar 20, 2024
 *      Author: de
*/

#ifndef _SPICLIENT_H_INCLUDED_
#define _SPICLIENT_H_INCLUDED_

//////////////////////////// PLATFORM SPECIFIC /////////////////////////////////

#if defined(PLATFORM_STM32_HAL)

#if defined(HAL_SPI_MODULE_ENABLED)

class SPI
{
protected:
	SPI_HandleTypeDef *_hspi = nullptr;
	GPIO_TypeDef *Port = GPIOB;
	uint16_t Pin = 0;
protected:
	SPI(SPI_HandleTypeDef *hInstance) :
			_hspi(hInstance) {}
};

class SPIPolling : public SPI
{
public:
	SPIPolling(SPI_HandleTypeDef *hInstance) :
		SPI(hInstance)
	{
	}

	inline void selectDevice()
	{
		HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	}

	inline void unselectDevice()
	{
		HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);
	}

	inline bool write(uint8_t b, uint32_t timeout)
	{
		return write(&b, 1, timeout);
	}

	inline bool write(uint8_t *b, uint16_t size, uint32_t timeout)
	{
		selectDevice(); // low, selecting the device
		HAL_StatusTypeDef ok = HAL_SPI_Transmit(_hspi, b, size, timeout);
		unselectDevice(); // high, unselecting the device
		return HAL_OK == ok;
	}

	bool read(uint8_t *pBuffer, uint16_t size, uint32_t timeout)
	{
		selectDevice(); // low, selecting the device
		HAL_StatusTypeDef ok = HAL_SPI_Receive(_hspi, pBuffer, size, timeout);
		unselectDevice(); // high, unselecting the device
		return HAL_OK == ok;
	}

	bool writeRead(uint8_t *pWriteBuf, uint8_t *pReadBuf, uint16_t size, uint32_t timeout)
	{
		HAL_StatusTypeDef ok = HAL_SPI_TransmitReceive(_hspi, pWriteBuf, pReadBuf, size, timeout);
		return HAL_OK == ok;
	}
};

#else // SPI not enabled

// mock
class SPI
{
public:
	SPI(void *){}
};
// mock
class SPIPolling : public SPI
{
public:
	SPIPolling(void *p) :
		SPI(p)
	{
	}
};

#endif

#else
#error "Not supported platform"
#endif

/////////////////////// END OF PLATFORM SPECIFIC //////////////////////////////

// Base class for type checking
class SPIClientBase
{
public:
	uint32_t Timeout = 300;
};

template <class TSPI = SPIPolling>
class SPIClientImpl : public SPIClientBase
{
protected:
	TSPI *_spi = nullptr;
public:
	using InterfaceType = TSPI;
	using InterfacePtrType = TSPI*;
protected:
	SPIClientImpl(TSPI *spi) :
		_spi(spi)
	{
	}

	inline bool write(uint8_t b) const
	{
		return _spi->write(b, Timeout);
	}

	inline bool write(uint8_t *b, uint16_t size) const
	{
		return _spi->write(b, size, Timeout);
	}

	inline bool read(uint8_t *pBuffer, uint16_t size) const
	{
		return _spi->read(pBuffer, size, Timeout);
	}

	inline bool read(uint8_t *b) const
	{
		return _spi->read(b, 1, Timeout);
	}

	inline bool writeRead(uint8_t bWrite, uint8_t &bRead)
	{
		return _spi->writeRead(&bWrite, &bRead, 1, Timeout);
	}

	inline void selectDevice()
	{
		return _spi->selectDevice();
	}

	inline void unselectDevice()
	{
		return _spi->unselectDevice();
	}
};

using SPIClientPolling = SPIClientImpl<SPIPolling>;

#endif // _SPICLIENT_H_INCLUDED_
