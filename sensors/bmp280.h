/*
 * bmp280.h
 *
 *  Created on: Mar 18, 2024
 *      Author: de
 */

#ifndef _BMP280_H_INCLUDED_
#define _BMP280_H_INCLUDED_

#include <delay.h>
#include <i2cclient.h>

enum class BMP280Error : uint8_t
{
	OK = 0,
	NotInitialized = 1,
	StartupTimeExceeded = 2,
	UnableToWriteResetCommand = 3,
	NotSupportedChipID = 4,
	InvalidChipID = 5,
	Busy = 6,
	UnableToQueryReadyStatus = 7,
	UnableToReadReadyStatus = 8,
	UnableToReadCompensationCoefficients = 9,
};

// The 7-bit device address is 111011x. The 6 MSB bits are fixed. The last bit is changeable by SDO
// value and can be changed during operation. Connecting SDO to GND results in slave address
// 1110110 (0x76); connection it to VDDIO results in slave address 1110111 (0x77), which is the same
// as BMP180’s I2C address. The SDO pin cannot be left floating; if left floating, the I2C address will
// be undefined.
enum class BMP280I2CAddress : uint8_t
{
	// SDO->GND
	SDOToGND = 0b0111'0110 << 1, // The Right bit is R/W bit in I2C
	// SDO->VDDIO
	SDOToVDDIO = 0b0111'0111 << 1,
};

// In sleep mode, no measurements are performed. Normal mode comprises an automated
// perpetual cycling between an active measurement period and an inactive standby period. In
// forced mode, a single measurement is performed. When the measurement is finished, the sensor
// returns to sleep mode.
enum class BMP280PowerMode : uint8_t
{
	Sleep = 0b00,
	Normal = 0b01,
	Forced = 0b10,
};

// Pressure measurement can be enabled or skipped. Skipping the measurement could be useful if
// BMP280 is used as temperature sensor. When enabled, several oversampling options exist. Each
// oversampling step reduces noise and increases the output resolution by one bit, which is stored
// in the XLSB data register 0xF9. Enabling/disabling the measurement and oversampling settings
// are selected through the osrs_p[2:0] bits in control register 0xF4.
// see page 12
enum class BMP280PressureOversampling : uint8_t
{
	Skipped = 0, // output set to 0x80000
	UltraLowPower,
	LowPower,
	StandardResolution,
	HighResolution,
	UltraHighResolution,
};

// Temperature measurement can be enabled or skipped. Skipping the measurement could be
// useful to measure pressure extremely rapidly. When enabled, several oversampling options exist.
// Each oversampling step reduces noise and increases the output resolution by one bit, which is
// stored in the XLSB data register 0xFC. Enabling/disabling the temperature measurement and
// oversampling setting are selected through the osrs_t[2:0] bits in control register 0xF4.
enum class BMP280TemperatureOversampling : uint8_t
{
	Skipped = 0,
	UltraLowPower,
	LowPower,
	StandardResolution,
	HighResolution,
	UltraHighResolution,
};

// The environmental pressure is subject to many short-term changes, caused e.g. by slamming of
// a door or window, or wind blowing into the sensor. To suppress these disturbances in the output
// data without causing additional interface traffic and processor work load, the BMP280 features
// an internal IIR filter. It effectively reduces the bandwidth of the output signals.
// The IIR filter can be configured using the filter[2:0] bits in control register 0xF5
// See page 13.
enum class BMP280IIRFilterMode : uint8_t
{
	FilterOff = 0,
	Filter2 = 2,
	Filter4 = 4,
	Filter8 = 8,
	Filter16 = 16,
};

// For normal mode, see page 16
// time cycle = osrs_t + osrs_p + standby
// In t_standby we can read the data.
// The standby time is determined by the contents of the t_sb[2:0] bits in control register 0xF5
enum class BMP280StandbyTime : uint16_t
{
	Standby_0_5 = 0b000,  // 0.5 ms
	Standby_62_5 = 0b001, // 62.5 ms
	Standby_125 = 0b010,  // 125 ms
	Standby_250 = 0b011,  // 250 ms
	Standby_500 = 0b100,  // 500
	Standby_1000 = 0b101, // 1000
	Standby_2000 = 0b110, // 2000
	Standby_4000 = 0b111, // 4000
};

/**
 * BMP280
 * DIGITAL PRESSURE SENSOR
 */
class BMP280 : public I2CClient
{
public:
	// Measurement time, see page 18
	enum MeasurementTime: uint16_t
	{
		StartupTimeMS = 2,
		UltraLowPowerTimeMS = 7, // max 6.4
		LowPowerTimeMS = 9, // max 8.7
		StandardPowerTimeMS = 14, // max 13.3
		HighResolutionTimeMS = 23, // max 22.5
		UltraHighResolutionTimeMS = 44, // max 43.2
	};
	// IDs etc.
	enum : uint8_t {
		INVALID_CHIP_ID = 0xFF,
		BMP280_CHIP_ID = 0x58, // for BMP280 must be 0x58
		OVERSAMPLING_REGISTRY = 0xF4,
		FILTER_REGISTRY = 0xF5,
		START_READ_DATA_REGISTRY = 0xF7, // read from F7 to FA in BMP280
	};
private:
	// All values are sets to 0 when reset (see 4.2 Memory map at page 24)
	mutable uint32_t _lastUpdateTime = 0;
	uint8_t _chipId = INVALID_CHIP_ID;
	bool _initialized = false;
	mutable BMP280Error _error = BMP280Error::NotInitialized;
	BMP280IIRFilterMode _filterMode = BMP280IIRFilterMode::FilterOff;
	BMP280PowerMode _mode = BMP280PowerMode::Sleep;
	BMP280PressureOversampling _pos = BMP280PressureOversampling::Skipped;
	BMP280TemperatureOversampling _tos = BMP280TemperatureOversampling::Skipped;
	BMP280StandbyTime _standbyTime = BMP280StandbyTime::Standby_0_5;
public:
	BMP280(I2C *i2c, BMP280I2CAddress addr = BMP280I2CAddress::SDOToGND)
	: I2CClient(i2c, (uint16_t)addr)
	{
		
	}

	inline BMP280Error error() const
	{
		return _error;
	}
	/**
	 * Returns the pressure (Pa) or UINT32_MAX if an error was occurred.
	 */
	uint32_t pressure() const
	{
		uint32_t p = UINT32_MAX;
		return p;
	}

	uint32_t getWaitTimeMS() const
	{
		uint32_t tt = 0, tp = 0;
		switch (_tos)
		{
		case BMP280TemperatureOversampling::Skipped:
			break;
		case BMP280TemperatureOversampling::HighResolution:
			tt = (uint8_t)MeasurementTime::HighResolutionTimeMS;
			break;
		case BMP280TemperatureOversampling::LowPower:
			tt = (uint8_t)MeasurementTime::LowPowerTimeMS;
			break;
		case BMP280TemperatureOversampling::StandardResolution:
			tt = (uint8_t)MeasurementTime::StandardPowerTimeMS;
			break;
		case BMP280TemperatureOversampling::UltraHighResolution:
			tt = (uint8_t)MeasurementTime::UltraHighResolutionTimeMS;
			break;
		case BMP280TemperatureOversampling::UltraLowPower:
			tt = (uint8_t)MeasurementTime::UltraLowPowerTimeMS;
			break;
		}
		switch (_pos)
		{
		case BMP280PressureOversampling::Skipped:
			break;
		case BMP280PressureOversampling::HighResolution:
			tp = (uint8_t)MeasurementTime::HighResolutionTimeMS;
			break;
		case BMP280PressureOversampling::LowPower:
			tp = (uint8_t)MeasurementTime::LowPowerTimeMS;
			break;
		case BMP280PressureOversampling::StandardResolution:
			tp = (uint8_t)MeasurementTime::StandardPowerTimeMS;
			break;
		case BMP280PressureOversampling::UltraHighResolution:
			tp = (uint8_t)MeasurementTime::UltraHighResolutionTimeMS;
			break;
		case BMP280PressureOversampling::UltraLowPower:
			tp = (uint8_t)MeasurementTime::UltraLowPowerTimeMS;
			break;
		}
		return tt > tp ? tt : tp; // maximum
	}

	bool enableSPI3Interface(bool bEnable)
	{
		uint8_t f5 = 0;
		if (!getControlRegistry(FILTER_REGISTRY, f5))
			return false;
		f5 &= 0b11111110; // bit 1 not used, clear bit 0
		if (bEnable)
			f5 |= 1; // 0 bit of 0xF5 registry
		if (!setControlRegistry(FILTER_REGISTRY, f5))
			return false;
		_lastUpdateTime = Delay::millis();
		return true;
	}

	bool setIIRFilter(BMP280IIRFilterMode m)
	{
		uint8_t f5 = 0;
		if (!getControlRegistry(FILTER_REGISTRY, f5))
			return false;
		f5 &= 0b11100011;
		f5 |= ((uint8_t)m) << 2; // 4, 3, 2 bits of 0xF5 registry
		if (!setControlRegistry(FILTER_REGISTRY, f5))
			return false;
		_lastUpdateTime = Delay::millis();
		_filterMode = m;
		return true;
	}

	bool setStandbyTime(BMP280StandbyTime t)
	{
		uint8_t f5 = 0;
		if (!getControlRegistry(FILTER_REGISTRY, f5))
			return false;
		f5 &= 0b00011111;
		f5 |= ((uint8_t)t) << 5; // 7, 6, 5 bits of 0xF5 registry
		if (!setControlRegistry(FILTER_REGISTRY, f5))
			return false;
		_lastUpdateTime = Delay::millis();
		return true;

	}

	bool setF5Registry(BMP280TemperatureOversampling t,
			BMP280PressureOversampling p, BMP280PowerMode m)
	{
		if (!_initialized)
			return false;
		if (!Delay::exceeded(_lastUpdateTime, getWaitTimeMS()))
			return false;
		// page 24
		// F4 control register:
		// osrs_t[2:0] | osrs_p[2:0] | mode[1:0]
		uint8_t cmd[2] = {0xF4, 0};
		cmd[1] = (((uint8_t) t) << 5) |  // temperature
				 (((uint8_t) p) << 2) |  // pressure
				 ((uint8_t) m);          // power mode
		if (!write(cmd, sizeof(cmd)))
			return false;
		return true;
	}

	inline bool setControlRegistry(uint8_t registry, uint8_t newValue)
	{
		uint8_t cmd[2] = {registry, newValue};
		return write(cmd, sizeof(cmd));
	}

	inline bool getControlRegistry(uint8_t registry, uint8_t &outValue)
	{
		if (!write(registry))
			return false;
		return read(&outValue);
	}

	bool setTemperatureOversampling(BMP280TemperatureOversampling value)
	{
		uint8_t f4 = 0;
		if (!getControlRegistry(OVERSAMPLING_REGISTRY, f4))
			return false;
		f4 &= 0b000'111'11; // temperature: 7,6,5 bits
		f4 |= (((uint8_t)value) & 0b111) << 5;
		_tos = value;
		return setControlRegistry(OVERSAMPLING_REGISTRY, f4);
	}

	bool setPressureOversampling(BMP280PressureOversampling value)
	{
		uint8_t f4 = 0;
		if (!getControlRegistry(OVERSAMPLING_REGISTRY, f4))
			return false;
		f4 &= 0b111'000'11; // pressure: 4,3,2 bits
		f4 |= (((uint8_t)value) & 0b111) << 2;
		_pos = value;
		return setControlRegistry(OVERSAMPLING_REGISTRY, f4);
	}

	bool setPowerMode(BMP280PowerMode value)
	{
		uint8_t f4 = 0;
		if (!getControlRegistry(OVERSAMPLING_REGISTRY, f4))
			return false;
		f4 &= 0b111'111'00; // power mode: 1, 0 bits
		f4 |= ((uint8_t)value & 0b11);
		_mode = value;
		return setControlRegistry(OVERSAMPLING_REGISTRY, f4);
	}


	/**
	 * Returns the chip ID
	 *
	 * The “id” register contains the chip identification number chip_id[7:0],
	 * which is 0x58. This number can be read as soon as the device finished
	 * the power-on-reset. (page 24)
	 *
	 * @returns The chip ID (usually 0x58) or 0xFF if error was occurred.
	 */
	uint8_t chipId() const
	{
		uint8_t cmd = 0xD0; // get ID
		if (!write(&cmd, sizeof(cmd)))
			return INVALID_CHIP_ID;
		if (!read(&cmd, 1))
			return INVALID_CHIP_ID;
		return cmd;
	}

	bool _initStarted = false;

	bool init()
	{
		_initStarted = true;
		_error = BMP280Error::OK; // will be set later if error was occurred
		_initialized = false;
		if (!reset()) {
			_initStarted = false;
			return false;
		}
		_chipId = chipId();
		if (_chipId == INVALID_CHIP_ID) {
			_error = BMP280Error::InvalidChipID;
			_initStarted = false;
			return false;
		}
		if (_chipId != BMP280_CHIP_ID) {
			_error = BMP280Error::NotSupportedChipID;
			_initStarted = false;
			return false;
		}
		if (!readCompensationCoefficients()) {
			_initStarted = false;
			return false;
		}
		_initialized = true;
		_initStarted = false;
		return true;
	}

	/**
	 * Resets the sensor
	 *
	 * Waits some time for 0xF3 registry ready bits
	 *
	 * The “reset” register contains the soft reset word reset[7:0].
	 * If the value 0xB6 is written to the register, the device is reset using
	 * the complete power-on-reset procedure. (page 24)
	 */
	bool reset()
	{
		_error = BMP280Error::OK;
		// The RESET command: REG, DATA
		uint8_t cmd[2] = {0x0E, 0xB6};
		bool ok = write(cmd, sizeof(cmd));
		if (!ok) {
			_error = BMP280Error::UnableToWriteResetCommand;
			return false;
		}
		// wait for ready: 0xF3 registry
		uint32_t currentTime = Delay::millis();
		while (!ready()) {
			if (_error != BMP280Error::OK) // error inside of ready()
				return false;
			// prevent the endless cycle
			if (Delay::exceeded(currentTime, StartupTimeMS)) {
				_error = BMP280Error::StartupTimeExceeded;
				return false;
			}
		}
		return true;
	}

	bool ready() const
	{
		_error = BMP280Error::OK;
		if (!_initialized) {
			if (!_initStarted) {
				_error = BMP280Error::NotInitialized;
				return false;
			}
			// OK, init() now is called, continue init() function
		}
		if (!Delay::exceeded(_lastUpdateTime, getWaitTimeMS())) {
			_error = BMP280Error::Busy;
			return false;
		}
		uint8_t status = 0xF3; // Register 0xF3 “status”
		bool ok = write(status);
		if (!ok) {
			_error = BMP280Error::UnableToQueryReadyStatus;
			return false;
		}
		ok = read(&status);
		if (!ok) {
			_error = BMP280Error::UnableToReadReadyStatus;
			return false;
		}
		// bit 1: sets 0 if copying is done
		// bit 3: conversion is running -> 1
		return !(status & 0b1001);
	}

	bool readCompensationCoefficients()
	{
		//The trimming parameters are programmed into the devices’ non-volatile memory (NVM) during
		//production and cannot be altered by the customer. Each compensation word is a 16-bit signed or
		//unsigned integer value stored in two’s complement. As the memory is organized into 8-bit words,
		//two words must always be combined in order to represent the compensation word. The 8-bit
		//registers are named calib00...calib25 and are stored at memory addresses 0x88...0xA1.
		// (page 21)
		uint8_t buf[26] = {};
		write(0x88);
		if (!read(buf, sizeof(buf))) {
			_error = BMP280Error::UnableToReadCompensationCoefficients;
			return false;
		}
		// LSB - 0, MSB - 1
		cdata.dig_T1 = (buf[1] << 8) | buf[0];
		cdata.dig_T2 = (buf[3] << 8) | buf[2];
		cdata.dig_T3 = (buf[5] << 8) | buf[4];
		cdata.dig_P1 = (buf[7] << 8) | buf[6];
		cdata.dig_P2 = (buf[9] << 8) | buf[8];
		cdata.dig_P3 = (buf[11] << 8) | buf[10];
		cdata.dig_P4 = (buf[13] << 8) | buf[12];
		cdata.dig_P5 = (buf[15] << 8) | buf[14];
		cdata.dig_P6 = (buf[17] << 8) | buf[16];
		cdata.dig_P7 = (buf[19] << 8) | buf[18];
		cdata.dig_P8 = (buf[21] << 8) | buf[20];
		cdata.dig_P9 = (buf[23] << 8) | buf[22];
		cdata.reserved = (buf[25] << 8) | buf[24];
		return true;
	}

	/**
	 * Reads the temperature and pressure data
	 *
	 * Temperature: signed 32 bit integer value with two last fractional digits: 12345 -> 123.45
	 * Pressure: unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	 */
	bool readData(int32_t &outTemperature, uint32_t &outPressure)
	{
		// To read out data after a conversion, it is strongly recommended to use a burst read and not
		// address every register individually. This will prevent a possible mix-up of bytes belonging to
		// different measurements and reduce interface traffic. Data readout is done by starting a burst read
		// from 0xF7 to 0xFC. The data are read out in an unsigned 20-bit format both for pressure and for
		// temperature.
		uint8_t buf[6] {};
		if (!write(START_READ_DATA_REGISTRY))
			return false;
		uint8_t n = 6; // TODO: set 8 for BME280 in future
		if (!read(buf, n))
			return false;
		// see page 26-27
		// Register 0xF7...0xF9 “press” (_msb, _lsb, _xlsb)
		// MSB LSB XLSB
		uint32_t adc_p = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
		uint32_t adc_t = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
		BMP280_S32_t t_fine = 0;
		outTemperature =  bmp280_compensate_T_int32(adc_t, t_fine);
		outPressure = bmp280_compensate_P_int32(adc_p, t_fine);
		return true;
	}


private:
	using BMP280_S32_t = int32_t;
	using BMP280_U32_t = uint32_t;
	using BMP280_S64_t = int64_t;
	// see page 21
	// Compensation parameter storage, naming and data type; LSB/MSB
	struct {
		uint16_t dig_T1; // 0x88 / 0x89, unsigned short
		int16_t dig_T2;  // 0x8A / 0x8B, signed short
		int16_t dig_T3; // 0x8C / 0x8D, signed short
		uint16_t dig_P1; // 0x8E / 0x8F, unsigned short
		int16_t dig_P2; // 0x90 / 0x91, signed short
		int16_t dig_P3; // 0x92 / 0x93, signed short
		int16_t dig_P4; // 0x94 / 0x95, signed short
		int16_t dig_P5; // 0x96 / 0x97, signed short
		int16_t dig_P6; // 0x98 / 0x99, signed short
		int16_t dig_P7; // 0x9A / 0x9B, signed short
		int16_t dig_P8; // 0x9C / 0x9D, signed short
		int16_t dig_P9; // 0x9E / 0x9F, signed short
		int16_t reserved; // 0xA0 / 0xA1, reserved
	} cdata {};

	// The next functions are from BMP280 datasheet as is.

	// see page 22
	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
	// t_fine carries fine temperature
	BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T, BMP280_S32_t &t_fine)
	{
		BMP280_S32_t var1, var2, T;
		var1 = ((((adc_T >> 3) - ((BMP280_S32_t) cdata.dig_T1 << 1)))
				* ((BMP280_S32_t) cdata.dig_T2)) >> 11;
		var2 = (((((adc_T >> 4) - ((BMP280_S32_t) cdata.dig_T1))
				* ((adc_T >> 4) - ((BMP280_S32_t) cdata.dig_T1))) >> 12)
				* ((BMP280_S32_t) cdata.dig_T3)) >> 14;
		t_fine = var1 + var2;
		T = (t_fine * 5 + 128) >> 8;
		return T;
	}
	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
	BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P, BMP280_S32_t &t_fine)
	{
		BMP280_S64_t var1, var2, p;
		var1 = ((BMP280_S64_t) t_fine) - 128000;
		var2 = var1 * var1 * (BMP280_S64_t) cdata.dig_P6;
		var2 = var2 + ((var1 * (BMP280_S64_t) cdata.dig_P5) << 17);
		var2 = var2 + (((BMP280_S64_t) cdata.dig_P4) << 35);
		var1 = ((var1 * var1 * (BMP280_S64_t) cdata.dig_P3) >> 8)
				+ ((var1 * (BMP280_S64_t) cdata.dig_P2) << 12);
		var1 = (((((BMP280_S64_t) 1) << 47) + var1)) * ((BMP280_S64_t) cdata.dig_P1)
				>> 33;
		if (var1 == 0)
		{
			return 0; // avoid exception caused by division by zero
		}
		p = 1048576 - adc_P;
		p = (((p << 31) - var2) * 3125) / var1;
		var1 = (((BMP280_S64_t) cdata.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
		var2 = (((BMP280_S64_t) cdata.dig_P8) * p) >> 19;
		p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t) cdata.dig_P7) << 4);
		return (BMP280_U32_t) p;
	}

	// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
	// t_fine carries fine temperature as global value
	double bmp280_compensate_T_double(BMP280_S32_t adc_T, BMP280_S32_t &t_fine)
	{
		double var1, var2, T;
		var1 = (((double) adc_T) / 16384.0 - ((double) cdata.dig_T1) / 1024.0)
				* ((double) cdata.dig_T2);
		var2 = ((((double) adc_T) / 131072.0 - ((double) cdata.dig_T1) / 8192.0)
				* (((double) adc_T) / 131072.0 - ((double) cdata.dig_T1) / 8192.0))
				* ((double) cdata.dig_T3);
		t_fine = (BMP280_S32_t) (var1 + var2);
		T = (var1 + var2) / 5120.0;
		return T;
	}
	// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
	double bmp280_compensate_P_double(BMP280_S32_t adc_P, BMP280_S32_t &t_fine)
	{
		double var1, var2, p;
		var1 = ((double) t_fine / 2.0) - 64000.0;
		var2 = var1 * var1 * ((double) cdata.dig_P6) / 32768.0;
		var2 = var2 + var1 * ((double) cdata.dig_P5) * 2.0;
		var2 = (var2 / 4.0) + (((double) cdata.dig_P4) * 65536.0);
		var1 = (((double) cdata.dig_P3) * var1 * var1 / 524288.0
				+ ((double) cdata.dig_P2) * var1) / 524288.0;
		var1 = (1.0 + var1 / 32768.0) * ((double) cdata.dig_P1);
		if (var1 == 0.0)
		{
			return 0; // avoid exception caused by division by zero
		}
		p = 1048576.0 - (double) adc_P;
		p = (p - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double) cdata.dig_P9) * p * p / 2147483648.0;
		var2 = p * ((double) cdata.dig_P8) / 32768.0;
		p = p + (var1 + var2 + ((double) cdata.dig_P7)) / 16.0;
		return p;
	}

	// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
	BMP280_U32_t bmp280_compensate_P_int32(BMP280_S32_t adc_P, BMP280_S32_t &t_fine)
	{
		BMP280_S32_t var1, var2;
		BMP280_U32_t p;
		var1 = (((BMP280_S32_t) t_fine) >> 1) - (BMP280_S32_t) 64000;
		var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((BMP280_S32_t) cdata.dig_P6);
		var2 = var2 + ((var1 * ((BMP280_S32_t) cdata.dig_P5)) << 1);
		var2 = (var2 >> 2) + (((BMP280_S32_t) cdata.dig_P4) << 16);
		var1 = (((cdata.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3)
				+ ((((BMP280_S32_t) cdata.dig_P2) * var1) >> 1)) >> 18;
		var1 = ((((32768 + var1)) * ((BMP280_S32_t) cdata.dig_P1)) >> 15);
		if (var1 == 0)
		{
			return 0; // avoid exception caused by division by zero
		}
		p = (((BMP280_U32_t) (((BMP280_S32_t) 1048576) - adc_P) - (var2 >> 12)))
				* 3125;
		if (p < 0x80000000)
		{
			p = (p << 1) / ((BMP280_U32_t) var1);
		}
		else
		{
			p = (p / (BMP280_U32_t) var1) * 2;
		}
		var1 = (((BMP280_S32_t) cdata.dig_P9)
				* ((BMP280_S32_t) (((p >> 3) * (p >> 3)) >> 13))) >> 12;
		var2 = (((BMP280_S32_t) (p >> 2)) * ((BMP280_S32_t) cdata.dig_P8)) >> 13;
		p = (BMP280_U32_t) ((BMP280_S32_t) p + ((var1 + var2 + cdata.dig_P7) >> 4));
		return p;
	}
};

#endif // _BMP280_H_INCLUDED_
