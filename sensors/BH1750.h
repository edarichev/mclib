/*
 * bh1750.h
 *
 *  Created on: Mar 17, 2024
 *      Author: de
 */

#ifndef INC_BH1750_H_
#define INC_BH1750_H_

#include <delay.h>
#include <i2cclient.h>
#include <cstring>

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

enum class BH1750Error : uint8_t
{
    OK = 0,
    UnableToQueryMTReg = 1, // thestage of query MTReg value failed
    UnableToReadMTReg = 2, // the stage of read MTReg value failed
    UnableToWriteMTReg = 3,
    UnableToWritePowerState = 4,
    UnableToWriteResolutionMode = 5,
    NotReady = 6, // try later
    UnableToReadRawData = 7,
    Uninitialized = 8,
};

/**
 * BH1750FVI
 * Digital 16bit Serial Output Type Ambient Light Sensor IC
 */
template<class TInterfaceClient = I2CClientPolling>
class BH1750Impl: public TInterfaceClient
{
protected:
    // The time it takes to change modes, for example, setting Resolution Mode,
    // MT register. Selected experimentally. May appear in OneTimeMode.
    enum : uint32_t
    {
        ChangeModeWaitTimeMs = 50,
        MeasureModeLowWaitTime = 24,
        MeasureModeHighWaitTime = 180,
        MTRegDefaultValue = 69
    };
    static const constexpr BH1750ResolutionMode DefaultResolutionMode =
            BH1750ResolutionMode::ContinuouslyHigh1;
protected:
    BH1750ResolutionMode _mode = DefaultResolutionMode;
    mutable uint32_t _lastUpdateTime = 0;
    uint8_t _mtValue = MTRegDefaultValue; // default value, [31...254]
    bool _initialized = false;
    bool _initStarted = false;
    bool _modeChanged = false;
    bool _mtRegChanged = false;
    mutable BH1750Error _error = BH1750Error::OK;
public:
    BH1750Impl(typename TInterfaceClient::InterfacePtrType i2c, uint16_t addr,
            BH1750ResolutionMode mode = DefaultResolutionMode)
            : TInterfaceClient(i2c, addr), _mode(mode)
    {

    }

    BH1750Error error() const
    {
        return _error;
    }

    uint8_t mtValue() const
    {
        return _mtValue;
    }
    /**
     * Sets the MTReg (measurement time register) value
     *
     * @param mt MTReg value [31; 254], default: 69
     */
    bool setMTReg(uint8_t mt)
    {
        if (!_initialized && !_initStarted) {
            _error = BH1750Error::Uninitialized;
            return false;
        }
        if (mt < 31)
            mt = 31;
        if (mt > 254)
            mt = 254;
        _error = BH1750Error::OK;
        _modeChanged = true;
        // 1. Change High bits: 7,6,5
        uint8_t v = mt >> 5;
        v |= 0b01000'000; // Mask for high bits
        bool ok = TInterfaceClient::write(v); // Send high bits
        if (!ok) {
            _error = BH1750Error::UnableToQueryMTReg;
            return false;
        }
        // 2. Change Low bits: 4,3,2,1,0
        v &= 0b11111;
        v |= 0b011'00000; // Mask for low bits
        ok = TInterfaceClient::write(v); // Send low bits
        if (!ok) {
            _error = BH1750Error::UnableToWriteMTReg;
            return false;
        }
        _mtValue = mt;
        _lastUpdateTime = Delay::millis();
        return true;
    }

    /**
     * The main function for powerOn, powerDown, reset
     */
    inline bool setPowerState(BH1750PowerState s) const
    {
        if (!_initialized) {
            _error = BH1750Error::Uninitialized;
            return false;
        }
        _error = BH1750Error::OK;
        if (!TInterfaceClient::write((uint8_t) s)) {
            _error = BH1750Error::UnableToWritePowerState;
            return false;
        }
        return true;
    }

    inline bool powerOn() const
    {
        return setPowerState(BH1750PowerState::PowerOn);
    }

    inline bool powerDown() const
    {
        return setPowerState(BH1750PowerState::PowerDown);
    }

    inline bool reset() const
    {
        return setPowerState(BH1750PowerState::Reset);
    }

    /**
     * Sets the resolution mode and MTReg value
     */
    bool init()
    {
        _initStarted = true;
        bool ok = setResolution(_mode) && setMTReg(_mtValue);
        if (ok) {
            _initialized = true;
        }
        _initStarted = false;
        return ok;
    }

    bool setResolution(BH1750ResolutionMode mode)
    {
        if (!_initialized && !_initStarted) {
            _error = BH1750Error::Uninitialized;
            return false;
        }
        _error = BH1750Error::OK;
        bool ok = false;
        switch (mode) {
        case BH1750ResolutionMode::ContinuouslyHigh1:
        case BH1750ResolutionMode::ContinuouslyHigh2:
        case BH1750ResolutionMode::ContinuouslyLow:
        case BH1750ResolutionMode::OneTimeHigh1:
        case BH1750ResolutionMode::OneTimeHigh2:
        case BH1750ResolutionMode::OneTimeLow:
            ok = TInterfaceClient::write((uint8_t) mode);
            if (!ok) {
                _error = BH1750Error::UnableToWriteResolutionMode;
                break;
            }
            _mode = mode;
            _lastUpdateTime = Delay::millis();
            break;
        }
        return ok;
    }

    bool ready() const
    {
        if (!_initialized) {
            _error = BH1750Error::Uninitialized;
            return false;
        }
        _error = BH1750Error::OK;
        uint32_t currentTicks = Delay::millis();
        int64_t delta = currentTicks - _lastUpdateTime;
        if ((delta < 0) || (delta > getWaitTimeMs()))
            return true;
        _error = BH1750Error::NotReady;
        return false;
    }

    /**
     * Returns the value in luxes.
     *
     * If sensor is not ready, the return value may be incorrect,
     * please check first with ready() method.
     */
    float value() const
    {
        if (!_initialized) {
            _error = BH1750Error::Uninitialized;
            return false;
        }
        uint16_t lumen = 0;
        float result = -1; // (lx)
        // The result of continuously measurement mode is updated.

        // In one time measurement, Statement moves to power down mode after
        // measurement completion. If updated result is need
        // then please resend measurement instruction. (p. 7)
        // (i.e. call gy302.setResolution(OneTimeXXX) before next reading data)
        lumen = getRawData();
        // see page 11
        result = ((float) lumen * MTRegDefaultValue / 1.2f) / _mtValue;
        switch (_mode) {
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

    /**
     * Returns value in XXXYY form where XXX - integer part, YY - fractional part:
     * 12345 -> 123.45
     */
    uint32_t intValue() const
    {
        return (uint32_t) (value() * 100);
    }

protected:

    uint32_t getWaitTimeMs() const
    {
        uint32_t t = 0;
        if (!_initialized && ChangeModeWaitTimeMs > 0)
            return ChangeModeWaitTimeMs;
        switch (_mode) {
        case BH1750ResolutionMode::ContinuouslyLow:
        case BH1750ResolutionMode::OneTimeLow:
            t = MeasureModeLowWaitTime;
            break;
        default:
            t = MeasureModeHighWaitTime;
            break;
        }
        return (t * _mtValue) / MTRegDefaultValue;
    }

    uint16_t getRawData() const
    {
        if (!_initialized) {
            _error = BH1750Error::Uninitialized;
            return false;
        }
        _error = BH1750Error::OK;
        uint16_t value = 0;
        uint8_t buf[2] = { 0, 0 };
        if (!TInterfaceClient::read(buf, sizeof(buf))) {
            _error = BH1750Error::UnableToReadRawData;
            return 0xFFFF;
        }
        value = ((uint16_t) buf[0] << 8) | buf[1];
        return value;
    }
};

enum class BH1750Addr : uint16_t
{
    // ADDR pin set to 0 or GND: 0x23
    ADDR_GND = 0x23 << 1, // ADDR from data sheet including R/W bit
    // ADDR pin set to 1 or VCC: 0x5C
    ADDR_VCC = 0x5C << 1,
};

class BH1750: public BH1750Impl<I2CClientPolling>
{
public:
    /**
     * Creates a new object for BH1750 Sensor
     *
     */
    BH1750(I2CPollingModeMaster *i2c, BH1750Addr addr = BH1750Addr::ADDR_GND)
            : BH1750Impl(i2c, (uint16_t) addr)
    {

    }
};

#endif /* INC_BH1750_H_ */
