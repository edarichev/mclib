/**
 * Misc draft functions for 24LC64 EEPROM
 * 
 * @author Evgeny Darichev
 * @date 28.03.2024
 */
#ifndef _24LC64_H_INCLUDED_
#define _24LC64_H_INCLUDED_

#include "../i2cclient.h"
#include "../delay.h"

#define EEPROM_24LC64_PATTERN_SIZE       16
#define EEPROM_24LC64_DEVICE_ADDR        0xA0
#define EEPROM_24LC64_MAX_ADDR           (0x1FFF + 1)

enum class EEPROM24LC64Error : uint8_t
{
    OK             = 0,
    UnableToWrite  = 1,
    WaitReadyTimeExceeded = 2,
    UnableToRead   = 3,
    PatternNotMatch = 4,
};

/**
 * Represents the 24LC64 EEPROM chip
 *
 * Note that writes to this chip are in 32-byte blocks, and you can get
 * mismatches when writing and reading to random locations.
 */
template<class TInterfaceClient = I2CPollingClient>
class EEPROM24LC64Impl : public TInterfaceClient
{
    mutable EEPROM24LC64Error _error = EEPROM24LC64Error::OK;
public:
    static const constexpr uint16_t MAX_ADDR = (0x1FFF + 1);
public:
    EEPROM24LC64Impl(typename TInterfaceClient::InterfacePtrType i2c, uint16_t addr)
        : TInterfaceClient(i2c, addr)
    {

    }

    EEPROM24LC64Error error() const
    {
        return _error;
    }

    bool write(uint16_t memAddr, uint8_t *buf, uint16_t size) const
    {
        _error = EEPROM24LC64Error::OK;
        if (!TInterfaceClient::memWrite(memAddr, 16, buf, size)) {
            _error = EEPROM24LC64Error::UnableToWrite;
            return false;
        }
        return true;
    }

    bool read(uint16_t memAddr, uint8_t *buf, uint16_t size) const
    {
        if (!TInterfaceClient::memRead(memAddr, 16, buf, size)) {
            _error = EEPROM24LC64Error::UnableToRead;
            return false;
        }
        return true;
    }

    /**
     * A memory test for 24LC64
     *
     * @param testChar test character for pattern, for example 0xAA or 0x55.
     * @returns true if test passed
     */
    bool memtest(uint8_t testChar) const
    {
        _error = EEPROM24LC64Error::OK;
        const uint8_t PATTERN_SIZE = 16;
        uint8_t wmsgPattern[PATTERN_SIZE];
        memset(wmsgPattern, testChar, PATTERN_SIZE);
        uint8_t buf[PATTERN_SIZE];
        uint32_t maxWaitTime = 1000;
        for (int addr = 0; addr < MAX_ADDR - PATTERN_SIZE; addr += PATTERN_SIZE) {
            // random access
            if (!write(addr, wmsgPattern, PATTERN_SIZE))
                return false; // error sets inside
            uint32_t t0 = Delay::millis();
            while (!ready()) {
                if (Delay::exceeded(t0, maxWaitTime)) {
                    // prevent endless cycle
                    _error = EEPROM24LC64Error::WaitReadyTimeExceeded;
                    return false;
                }
            }
            memset(buf, 0, PATTERN_SIZE);
            if (!read(addr, buf, PATTERN_SIZE))
                return false; // error sets inside
            if (memcmp(wmsgPattern, buf, PATTERN_SIZE) != 0) {
                _error = EEPROM24LC64Error::PatternNotMatch;
                return false;
            }
        }
        return true; // no errors -> true, test passed
    }

    bool ready() const
    {
        return TInterfaceClient::isDeviceReady();
    }
};

using EEPROM24LC64 = EEPROM24LC64Impl<I2CPollingClient>;

#endif // _24LC64_H_INCLUDED_
