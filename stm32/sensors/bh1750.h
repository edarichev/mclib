/*
 * bh1750.h
 *
 *  Created on: Mar 17, 2024
 *      Author: de
 */

#ifndef INC_BH1750_H_
#define INC_BH1750_H_

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

void error(const char *msg)
{
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

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

/**
 * BH1750  Power state
 *
 * See page 5.
 */
enum class BH1750PowerState : uint8_t
{
	// No active state.
	PowerDown = 0b0000'0000,
	// Waiting for measurement command.
	PowerOn = 0b0000'0001,
	// Reset Data register value. Reset command is not acceptable in Power Down mode.
	Reset = 0b0000'0111,
};

/**
 * BH1750 Resolution mode
 *
 * See page 5.
 */
enum class BH1750ResolutionMode : uint8_t
{
	// Start measurement at 1lx resolution.
	// Measurement Time is typically 120ms.
	ContinuouslyHigh1 = 0b0001'0000,

	// Start measurement at 0.5lx resolution.
	// Measurement Time is typically 120ms.
	ContinuouslyHigh2 = 0b0001'0001,

	// Start measurement at 4lx resolution.
	// Measurement Time is typically 16ms.
	ContinuouslyLow = 0b0001'0011,

	// Start measurement at 1lx resolution.
	// Measurement Time is typically 120ms.
	// It is automatically set to Power Down mode after measurement.
	OneTimeHigh1 = 0b0010'0000,

	// Start measurement at 0.5lx resolution.
	// Measurement Time is typically 120ms.
	// It is automatically set to Power Down mode after measurement.
	OneTimeHigh2 = 0b0010'0001,

	// Start measurement at 4lx resolution.
	// Measurement Time is typically 16ms.
	// It is automatically set to Power Down mode after measurement.
	OneTimeLow = 0b0010'0011,
};

class BH1750: public I2CClient
{
protected:
	// The time it takes to change modes, for example, setting Resolution Mode,
	// MT register. Selected experimentally. May appear in OneTimeMode.
	static const constexpr uint32_t ChangeModeWaitTimeMs = 50;
public:
	static const constexpr BH1750ResolutionMode DefaultResolutionMode =
			BH1750ResolutionMode::ContinuouslyHigh1;
	static const constexpr uint32_t MeasureModeLowWaitTime = 24;
	static const constexpr uint32_t MeasureModeHighWaitTime = 180;
	static const constexpr uint8_t MTRegDefaultValue = 69;
protected:
	BH1750ResolutionMode _mode = DefaultResolutionMode;
	mutable uint32_t _lastUpdateTime = 0;
	uint8_t _mtValue = MTRegDefaultValue; // default value, [31...254]
	mutable union {
		uint8_t _initialized : 1;
		uint8_t _modeChanged : 1;
		uint8_t _mtRegChanged : 1;
	} flags;
public:
	BH1750(I2C *i2c, uint16_t addr, BH1750ResolutionMode mode =
			DefaultResolutionMode) :
			I2CClient(i2c, addr), _mode(mode)
	{
		memset(&flags, 0, sizeof(flags));
	}

	/**
	 * Sets the MTReg value
	 *
	 * @param mt MTReg value [31; 254], default: 69
	 */
	bool setMTReg(uint8_t mt)
	{
		flags._modeChanged = 1;
		// 1. Change High bits: 7,6,5
		uint8_t v = mt >> 5;
		v |= 0b01000'000; // Mask for high bits
		bool ok = write(v); // Send high bits
		if (!ok)
			return false;
		// 2. Change Low bits: 4,3,2,1,0
		v &= 0b11111;
		v |= 0b011'00000; // Mask for low bits
		ok = ok && write(v); // Send low bits
		if (ok) {
			_mtValue = mt;
			_lastUpdateTime = Delay::millis();
		}
		return ok;
	}

	inline bool setPowerState(BH1750PowerState s) const
	{
		return write((uint8_t) s);
	}

	bool powerOn() const
	{
		return setPowerState(BH1750PowerState::PowerOn);
	}

	bool powerDown() const
	{
		return setPowerState(BH1750PowerState::PowerDown);
	}

	bool reset() const
	{
		return setPowerState(BH1750PowerState::Reset);
	}

	bool init()
	{
		bool ok = setResolution(_mode) && setMTReg(_mtValue);
		flags._initialized = true;
		return ok;
	}

	bool setResolution(BH1750ResolutionMode mode)
	{
		bool ok = false;
		switch (mode)
		{
		case BH1750ResolutionMode::ContinuouslyHigh1:
		case BH1750ResolutionMode::ContinuouslyHigh2:
		case BH1750ResolutionMode::ContinuouslyLow:
		case BH1750ResolutionMode::OneTimeHigh1:
		case BH1750ResolutionMode::OneTimeHigh2:
		case BH1750ResolutionMode::OneTimeLow:
			ok = write((uint8_t) mode);
			if (!ok)
				error("write res mode error");
			_mode = mode;
			_lastUpdateTime = Delay::millis();
			break;
		}
		return ok;
	}

protected:

	uint32_t getWaitTimeMs() const
	{
		uint32_t t = 0;
		if (!flags._initialized && ChangeModeWaitTimeMs)
			return ChangeModeWaitTimeMs;
		switch (_mode)
		{
		case BH1750ResolutionMode::ContinuouslyLow:
		case BH1750ResolutionMode::OneTimeLow:
			t = MeasureModeLowWaitTime;
		default:
			t = MeasureModeHighWaitTime;
		}
		return (t * _mtValue) / MTRegDefaultValue;
	}

	uint16_t getRawData() const
	{
		uint16_t value = 0;
		uint8_t buf[2] = {0, 0};
		if (!read(buf, sizeof(buf))) {
			error("read error");
			return 0xFFFF;
		}
		value = ((uint16_t)buf[0] << 8) | buf[1];
		return value;
	}

public:

	bool ready() const
	{
		uint32_t currentTicks = Delay::millis();
		int64_t delta = currentTicks - _lastUpdateTime;
		bool ok = (delta < 0) || (delta > getWaitTimeMs());
		if (ok) {
			memset(&flags, 0, sizeof(flags));
		}
		return ok;
	}

	/**
	 * Returns the value in luxes.
	 *
	 * If sensor is not ready, the return value may be incorrect,
	 * please check first with ready() method.
	 */
	float value() const
	{
		uint16_t lumen = 0;
		float result = -1; // (lx)
		// The result of continuously measurement mode is updated.

		// In one time measurement, Statement moves to power down mode after
		// measurement completion. If updated result is need
		// then please resend measurement instruction. (p. 7)
		// (i.e. call gy302.setResolution(OneTimeXXX) before next reading data)
		lumen = getRawData();
		// see page 11
		result = ((float)lumen * MTRegDefaultValue / 1.2f) / _mtValue;
		switch (_mode)
		{
		case BH1750ResolutionMode::ContinuouslyHigh2:
		case BH1750ResolutionMode::OneTimeHigh2:
			result /= 2;
			break;
		default:
			break;
		}
		_lastUpdateTime = Delay::millis();
		return result;
	}


};

/////////////// The GY-302 Board with BH1750 sensor ///////////////////////////

enum class GY302Addr : uint16_t
{
	// ADDR pin set to 0 or GND: 0x23
	ADDR_GND = 0x23,
	// ADDR pin set to 1 or VCC: 0x5C
	ADDR_VCC = 0x5C,
};

class GY302: public BH1750
{
public:
	/**
	 * Creates a new object for GY-302 Sensor
	 *
	 * @param addr The I2C address specified in data sheet, as is, do not shift left.
	 */
	GY302(I2C *i2c, GY302Addr addr = GY302Addr::ADDR_GND) :
			BH1750(i2c, ((uint16_t) addr << 1)) // ADDR from data sheet including R/W bit
	{

	}
};

#endif /* INC_BH1750_H_ */

/*
Example for OneTimeModes:


  I2C i2cInstance(&hi2c1);
  GY302 gy302(&i2cInstance);

  gy302.init();

  HAL_UART_Transmit(&huart2, (uint8_t*) "START\r\n", 7, HAL_MAX_DELAY);
  gy302.setResolution(BH1750ResolutionMode::OneTimeHigh2);
  gy302.setMTReg(BH1750::MTRegDefaultValue * 2);
  HAL_Delay(200);

  while (1)
  {
	  if (gy302.ready()) {
		  float lx = gy302.value();
		  char msg[80];
		  sprintf(msg, "%f (lx)\r\n", lx);
		  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		  HAL_Delay(1000);
		  // resend measurement instruction
		  // for continuous modes not required:
		  gy302.setResolution(BH1750ResolutionMode::OneTimeHigh2);
		  gy302.setMTReg(BH1750::MTRegDefaultValue * 2);
	  }
  }

 */
