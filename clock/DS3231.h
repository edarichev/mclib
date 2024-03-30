/**
 * DS3231 Real time clock
 * 
 * @author Evgeny Darichev
 * @date 28.03.2024
 */

#ifndef _DS3231_H_INCLUDED_
#define _DS3231_H_INCLUDED_

#include "../i2cclient.h"
#include <ctime>
#include <climits>

struct DS3231DateTime
{
public:
	uint8_t second = 0;
	uint8_t minute = 0;
	uint8_t hour = 0;
	// Values that correspond to the day of week are userdefined but must be sequential
	// (i.e., if 1 equals Sunday, then 2 equals Monday, and so on). Illogical
	// time and date entries result in undefined operation.
	uint8_t dayOfWeek = 0;
	uint8_t day = 0;
	uint8_t month = 0;
	uint8_t year = 0;
	uint8_t century = 0;
	// if 12-hour format: 1-pm, 0-am
	bool pm = false;
	// time in 12-hour format
	bool asAMPM = false;

	void toAMPM()
	{
		if (asAMPM)
			return; // already AMPM
		asAMPM = 1;
		pm = hour > 12 ? 1 : 0;
		if (hour > 12)
			hour -= 12;
	}

	void to24()
	{
		if (!asAMPM)
			return;
		asAMPM = 0;
		if (pm)
			hour += 12;
		pm = 0;
	}
};


#define FROM_BCD(x) (((0b11110000 & (x)) >> 4) * 10 + (0b00001111 & (x)))

#define TO_BCD(x) (((x) % 10) | (((x) / 10) << 4))

template <class TInterfaceClient = I2CClientPolling>
class DS3231Impl : public TInterfaceClient
{
public:
	// The blue board with CR2032 battery has two chips: DS3231M (clock) and
	// 24C32BN (EEPROM).
	// The DS3231S RTC chip’s fixed I2C address is 0x68, and the EEPROM’s 
	// default I2C address is 0x57 (though the address range is 0x50 to 0x57).
	// We can change addresses using A0, A1 and A2 on board.
	enum : uint8_t {
		DS3231_I2CAddress = 0x68,
		EEPROM_I2CAddress = 0x57,
		ERROR_VALUE = 0xFF,
	};
public:
	template <typename U = TInterfaceClient>
	DS3231Impl(typename U::InterfacePtrType i2c, uint8_t addr)
		: TInterfaceClient(i2c, (uint16_t) (addr << 1))
	{
		
	}
	
	DS3231DateTime dateTime()
	{
		DS3231DateTime t;
		uint8_t buf[7]; // 7 bytes for clock data
		if (!TInterfaceClient::memRead(0x00, buf, sizeof(buf))) {
			return t;
		}
		
		t.second = FROM_BCD(buf[0]);
		t.minute = FROM_BCD(buf[1]);
		uint8_t hb = buf[2];
		t.asAMPM = (hb & 0b0100'0000) ? 1 : 0;
		t.hour = FROM_BCD(0b0000'1111 & hb);
		uint8_t hadd = 0b0001'0000 & hb ? 10 : 0; // +10?
		t.pm = (t.asAMPM && (0b0010'0000 & hb)) ? 1 : 0;
		if (!t.asAMPM && 0b0010'0000 & hb) // 20-hour bit
			hadd = 20;
		t.hour += hadd;
		t.dayOfWeek = FROM_BCD(0b0000'0111 & buf[3]);
		t.day = FROM_BCD(buf[4]);
		uint8_t mc = buf[5];
		t.century = mc & 0b1000'0000 ? 1 : 0;
		t.month = FROM_BCD(mc & 0b0001'1111);
		t.year = FROM_BCD(buf[6]);
		return t;
	}

	bool setDateTime(const DS3231DateTime &t)
	{
		uint8_t buf[7];
		buf[0] = TO_BCD(t.second);
		buf[1] = TO_BCD(t.minute);
		buf[2] = packHour(t.hour, t.asAMPM, t.pm);
		buf[3] = t.dayOfWeek;
		buf[4] = TO_BCD(t.day);
		buf[5] = TO_BCD(t.month) | (t.century << 7);
		buf[6] = TO_BCD(t.year);
		if (!TInterfaceClient::memWrite(0x00, buf, sizeof(buf))) {
			return false;
		}
		return true;
	}

	bool switchToAMPM()
	{
		uint8_t reg;
		if (!getRegister(HOUR_REGISTER, reg))
			return false;
		if (0b0100'0000 & reg)
			return true; // already 12-hour format
		// otherwise 24 format is here
		reg = FROM_BCD(reg); // hour value
		bool pm = false;
		if (reg > 12) {
			reg -= 12;
			pm = true;
		}
		reg = packHour(reg, true, pm);
		return setRegister(HOUR_REGISTER, reg);
	}

	bool switchTo24()
	{
		uint8_t reg;
		if (!getRegister(HOUR_REGISTER, reg))
			return false;
		if (!(0b0100'0000 & reg))
			return true; // already 24-hour format
		bool pm = 0b0010'0000 & reg;
		reg = FROM_BCD(reg & 0b0011'1111);
		if (pm)
			reg += 12;
		reg = packHour(reg, false, false);
		return setRegister(HOUR_REGISTER, reg);
	}

	/**
	 * Returns seconds or 0xFF if error
	 */
	uint8_t second()
	{
		uint8_t reg;
		if (!getRegister(SECONDS_REGISTER, reg))
			return ERROR_VALUE;
		return FROM_BCD(reg);
	}

	uint8_t minute()
	{
		uint8_t reg;
		if (!getRegister(MINUTE_REGISTER, reg))
			return ERROR_VALUE;
		return FROM_BCD(reg);
	}

	/**
	 * Returns hour correspond to AM/PM or 24-hour mode. No 12/24 conversions performed.
	 */
	uint8_t hour()
	{
		uint8_t reg;
		if (!getRegister(HOUR_REGISTER, reg))
			return ERROR_VALUE;
		if ((reg & 0b0100'0000) == 0)
			return FROM_BCD(reg); // as is, 24 h mode
		return FROM_BCD(reg & 0b0001'1111);
	}

	/**
	 * If in 12-hour mode returns true if PM.
	 */
	bool pm()
	{
		uint8_t reg;
		if (!getRegister(HOUR_REGISTER, reg))
			return false;
		if ((reg & 0b0100'0000) == 0)
			return false; // as is, 24 h mode
		return reg & 0b0010'0000;
	}

	/**
	 * If in 12-hour mode returns true.
	 */
	bool asAMPM()
	{
		uint8_t reg;
		if (!getRegister(HOUR_REGISTER, reg))
			return false;
		return reg & 0b0100'0000;
	}

	uint8_t day()
	{
		uint8_t reg;
		if (!getRegister(DAYOFMONTH_REGISTER, reg))
			return ERROR_VALUE;
		return FROM_BCD(reg);
	}

	uint8_t dayOfWeek()
	{
		uint8_t reg;
		if (!getRegister(DAYOFWEEK_REGISTER, reg))
			return ERROR_VALUE;
		return FROM_BCD(reg);
	}

	uint8_t month()
	{
		uint8_t reg;
		if (!getRegister(MONTH_REGISTER, reg))
			return ERROR_VALUE;
		return FROM_BCD(0b0001'1111 & reg);
	}

	uint8_t year()
	{
		uint8_t reg;
		if (!getRegister(YEAR_REGISTER, reg))
			return ERROR_VALUE;
		return FROM_BCD(reg);
	}

	uint8_t century()
	{
		uint8_t reg;
		if (!getRegister(MONTH_REGISTER, reg))
			return ERROR_VALUE;
		return reg >> 7;
	}

	bool setHour(uint8_t hour, bool asAMPM, bool pm)
	{
		uint8_t reg = packHour(hour, asAMPM, pm);
		return setRegister(HOUR_REGISTER, reg);
	}

	bool setMinute(uint8_t minute)
	{
		return setRegister(MINUTE_REGISTER, TO_BCD(minute));
	}

	bool setSecond(uint8_t second)
	{
		return setRegister(SECONDS_REGISTER, TO_BCD(second));
	}

	bool setDay(uint8_t dayOfMonth)
	{
		return setRegister(DAYOFMONTH_REGISTER, TO_BCD(dayOfMonth));
	}

	bool setDayOfWeek(uint8_t dayOfWeek)
	{
		return setRegister(DAYOFWEEK_REGISTER, TO_BCD(dayOfWeek));
	}

	bool setMonth(uint8_t month)
	{
		uint8_t reg;
		if (!getRegister(MONTH_REGISTER, reg))
			return false;
		// save century bit
		reg &= 0b1000'0000;
		reg |= TO_BCD(month);
		return setRegister(MONTH_REGISTER, reg);
	}

	bool setYear(uint8_t year)
	{
		return setRegister(YEAR_REGISTER, TO_BCD(year));
	}

	/**
	 * Sets the century: 0 or 1 (what you want: 0 may be 19xx or 20xx years or other century)
	 */
	bool setCentury(uint8_t c)
	{
		uint8_t reg;
		if (!getRegister(MONTH_REGISTER, reg))
			return false;
		// save month bits
		reg &= 0b0001'1111;
		if (c)
			reg |= 0b1000'0000;
		return setRegister(MONTH_REGISTER, reg);
	}

	/**
	 * Sets the alarm once per second
	 *
	 * Select pin at board and connect it ot the SQW pin of DS3231 board.
	 * Create an interrupt handler and recall this method if you want use next
	 * interriupt from the timer.
	 */
	bool setAlarm1OncePerSecond()
	{
		// set bits A1M1==A1M2==A1M3==A1M4==1
		uint8_t buf[4] {0b1000'0000, 0b1000'0000, 0b1000'0000, 0b1000'0000};
		// nothing to set
		return updateAlarm1(buf);
	}

	/**
	 * Sets the alarm once per second
	 *
	 * Select pin at board and connect it ot the SQW pin of DS3231 board.
	 * Create an interrupt handler and recall this method if you want use next
	 * interriupt from the timer.
	 */
	bool setAlarm1SecondMatch(uint8_t second)
	{
		// set bits A1M1==0; A1M2==A1M3==A1M4==1
		uint8_t buf[4] {0b0000'0000, 0b1000'0000, 0b1000'0000, 0b1000'0000};
		buf[0] |= 0b0111'1111 & TO_BCD(second); // at specified second of every minute
		return updateAlarm1(buf);
	}

	bool setAlarm1MSMatch(uint8_t minute, uint8_t second)
	{
		// set bits A1M1==0; A1M2==0; A1M3==A1M4==1
		uint8_t buf[4] {0b0000'0000, 0b0000'0000, 0b1000'0000, 0b1000'0000};
		buf[0] = 0b0111'1111 & TO_BCD(second); // bit 7 must be 0
		buf[1] = 0b0111'1111 & TO_BCD(minute); // bit 7 must be 0
		// bits 7 of A1M3 & A1M4 already set
		return updateAlarm1(buf);
	}

	bool setAlarm1HMSMatch(uint8_t hour, uint8_t minute, uint8_t second,
			bool asAMPM = false, bool pm = false)
	{
		// set bits A1M1==0; A1M2==0; A1M3==0; A1M4==1
		uint8_t buf[4] {0b0000'0000, 0b0000'0000, 0b0000'0000, 0b1000'0000};
		buf[0] = 0b0111'1111 & TO_BCD(second); // bit 7 must be 0
		buf[1] = 0b0111'1111 & TO_BCD(minute); // bit 7 must be 0
		buf[2] = 0b0111'1111 & packHour(hour, asAMPM, pm);
		// bit 7 of A1M4 already set
		return updateAlarm1(buf);
	}

	bool setAlarm1DHMSMatch(uint8_t dayOfMonth, uint8_t hour, uint8_t minute,
			uint8_t second, bool asAMPM = false, bool pm = false)
	{
		// set bits A1M1==0; A1M2==0; A1M3==0; A1M4==1
		uint8_t buf[4];
		// all 7 bits must be 0
		buf[0] = 0b0111'1111 & TO_BCD(second);
		buf[1] = 0b0111'1111 & TO_BCD(minute);
		buf[2] = 0b0111'1111 & packHour(hour, asAMPM, pm);
		// bit 6 of date register must be set here to 0
		buf[3] = 0b0011'1111 & TO_BCD(dayOfMonth);
		// bit 7 of A1M4 already set
		return updateAlarm1(buf);
	}

	/**
	 * Sets alarm 1 when day of week, hours, minutes, and seconds match
	 *
	 * @param dayOfWeek 1 - Sunday, 2 - Monday, etc.
	 */
	bool setAlarm1WHMSMatch(uint8_t dayOfWeek, uint8_t hour, uint8_t minute,
			uint8_t second, bool asAMPM = false, bool pm = false)
	{
		// set bits A1M1==0; A1M2==0; A1M3==0; A1M4==1
		uint8_t buf[4];
		// all 7 bits must be 0
		buf[0] = 0b0111'1111 & TO_BCD(second);
		buf[1] = 0b0111'1111 & TO_BCD(minute);
		buf[2] = 0b0111'1111 & packHour(hour, asAMPM, pm);
		// bit 6 of DT/DY register must be set here to 1
		buf[3] = (0b0000'1111 & TO_BCD(dayOfWeek)) | 0b0100'0000;
		return updateAlarm1(buf);
	}

	/**
	 * Sets the Alarm 2 once per minute (00 seconds of every minute)
	 */
	bool setAlarm2OncePerMinute()
	{
		// set bits A2M2==A2M3==A2M4==1
		uint8_t buf[3] {0b1000'0000, 0b1000'0000, 0b1000'0000};
		// we set only flags
		return updateAlarm2(buf);
	}

	bool setAlarm2MinuteMatch(uint8_t minute)
	{
		// set bits A2M2==0; A2M3==A2M4==1
		uint8_t buf[3] {0b0000'0000, 0b1000'0000, 0b1000'0000};
		// we set only flags
		buf[0] = 0b0111'1111 & TO_BCD(minute);
		return updateAlarm2(buf);
	}

	bool setAlarm2HMMatch(uint8_t hour, uint8_t minute,
			bool asAMPM = false, bool pm = false)
	{
		// set bits A2M2=0; A2M3==0; A2M4==1
		uint8_t buf[3] {0b0000'0000, 0b0000'0000, 0b1000'0000};
		buf[0] = 0b0111'1111 & TO_BCD(minute); // bit 7 must be 0
		buf[1] = 0b1000'0000 | packHour(hour, asAMPM, pm); // bit 7 -> 1
		return updateAlarm2(buf);
	}

	bool setAlarm2DHMMatch(uint8_t dayOfMonth, uint8_t hour, uint8_t minute,
			bool asAMPM = false, bool pm = false)
	{
		// set bits A2M2=0; A2M3==0; A2M4==0
		uint8_t buf[3] {0b0000'0000, 0b0000'0000, 0b0000'0000};
		buf[0] = 0b0111'1111 & TO_BCD(minute); // bit 7 must be 0
		buf[1] = 0b0000'0000 | packHour(hour, asAMPM, pm); // bit 7 -> 1
		buf[2] = 0b0011'1111 & TO_BCD(dayOfMonth);
		return updateAlarm2(buf);
	}

	bool setAlarm2WHMMatch(uint8_t dayOfWeek, uint8_t hour, uint8_t minute,
			bool asAMPM = false, bool pm = false)
	{
		// set bits A2M2=0; A2M3==0; A2M4==0
		uint8_t buf[3] {0b0000'0000, 0b0000'0000, 0b0000'0000};
		buf[0] = 0b0111'1111 & TO_BCD(minute); // bit 7 must be 0
		buf[1] = 0b0000'0000 | packHour(hour, asAMPM, pm); // bit 7 -> 1
		buf[2] = 0b1000'0000 | (0b0000'0111 & TO_BCD(dayOfWeek));
		return updateAlarm2(buf);
	}

	/**
	 * Indicates that the oscillator either is stopped or was
	 * stopped for some period and may be used to judge the
	 * validity of the timekeeping data. (see page 14)
	 */
	bool oscillatorStopped()
	{
		uint8_t reg = 0;
		if (!getRegister(STATUS_REGISTER, reg))
			return true; // by default, see page 14
		return 0b1000'0000 & reg;
	}

	bool oscillatorEnabled()
	{
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return false;
		return 0b1000'0000 & reg;
	}

	bool setOscillatorEnabled()
	{
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return false;
		reg |= 0b1000'0000;
		return setRegister(CONTROL_REGISTER, reg);
	}

	bool setBatteryBackedSquareWaveEnabled(bool bEnable)
	{
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return false;
		if (bEnable)
			reg |= 0b0100'0000;
		else
			reg &= ~0b0100'0000;
		return setRegister(CONTROL_REGISTER, reg);
	}

	bool batteryBackedSquareWaveEnabled()
	{
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return false;
		return reg &= ~0b0100'0000;
	}

	/**
	 * CR bit 5: CONV.
	 * Setting this bit to 1 forces the temperature sensor to convert the
	 * temperature into digital code and execute the TCXO algorithm
	 * to update the capacitance array to the oscillator.
	 */
	bool setConvertTemperature(bool bSet)
	{
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return false;
		if (bSet)
			reg |= 0b0010'0000;
		else
			reg &= ~0b0010'0000;
		return setRegister(CONTROL_REGISTER, reg);
	}

	/**
	 * Checks CONV bit in CR.
	 */
	bool convertTemperatureEnabled()
	{
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return false;
		return reg &= ~0b0010'0000;
	}

	/**
	 * Bits 4, 3 of CR: These bits control the frequency of the square-wave
	 * output when the square wave has been enabled.
	 *
	 * @param r     rate: 0 - 1kHz, 2 - 1.024kHz, 3 - 4.096kHz, 4 - 8.192kHz
	 */
	bool setRate(uint8_t r)
	{
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return false;
		reg &= 0b1110'0111; // clear
		reg |= ((0b11 & r) << 3); // set
		return setRegister(CONTROL_REGISTER, reg);
	}

	/**
	 * Returns value of Bits 4, 3 of CR (Rate Select).
	 */
	uint8_t rate()
	{
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return ERROR_VALUE;
		return 0b11 & (reg >> 3);
	}

	bool alarm1Enabled()
	{
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return false;
		return reg | 0b0000'0001;
	}

	bool alarm2Enabled()
	{
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return false;
		return reg | 0b0000'0010;
	}

	bool setInterruptEnabled(bool bEnable)
	{
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return false;
		reg |= 0b0000'0100;
		return setRegister(CONTROL_REGISTER, reg);
	}

	bool interruptEnabled()
	{
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return false;
		return reg &= ~0b0000'0100;
	}

	/**
	 * Enables 32kHz pin output.
	 *
	 * @returns false if error.
	 */
	bool enable32kHz()
	{
		uint8_t reg = 0;
		if (!getRegister(STATUS_REGISTER, reg))
			return false;
		reg |= 0b0000'1000;
		return setRegister(STATUS_REGISTER, reg);
	}

	/**
	 * Checks 32kHz pin enabled. Returns false if not or error.
	 */
	bool is32kHzEnabled()
	{
		uint8_t reg = 0;
		if (!getRegister(STATUS_REGISTER, reg))
			return false;
		return reg | 0b0000'1000;
	}

	/**
	 * Returns BSY bit in control register.
	 *
	 * This bit indicates the device is busy
	 * executing TCXO functions. It goes to logic 1 when the
	 * conversion signal to the temperature sensor is asserted
	 * and then is cleared when the device is in the 1-minute
	 * idle state.
	 */
	bool busy()
	{
		uint8_t reg = 0;
		if (!getRegister(STATUS_REGISTER, reg))
			return false;
		return reg | 0b0000'0100;
	}
	bool setAging(int8_t a)
	{
		return setRegister(AGING_REGISTER, a);
	}

	int8_t aging()
	{
		uint8_t reg = 0;
		if (!getRegister(AGING_REGISTER, reg))
			return ERROR_VALUE;
		int8_t a = 0;
		a |= reg;
		return a;
	}

	/**
	 * Returns temperature in XXXYY signed integer format: XXX - integral part,
	 * YY - 2-digits fractional part. Divide by 100 to get fractional part.
	 *
	 * @returns INT16_MIN if error
	 */
	int16_t temperature()
	{
		int16_t msb = 0;
		uint8_t lsb = 0;
		uint8_t reg = 0;
		if (!getRegister(TEMPERATURE_REGISTER, reg))
			return INT16_MIN;
		msb |= reg;
		msb *= 100; // space for fractional part
		if (!getRegister(TEMPERATURE_REGISTER + 1, reg))
			return INT16_MIN;
		reg >>= 6; // data stored in 7 and 6 bits
		// fractional part - 1 as 0.25
		lsb = 25 * reg;
		return lsb + msb;
	}

private:
	/**
	 * Packs hour value into 1 byte (no 12/24 hour conversions)
	 */
	inline uint8_t packHour(uint8_t hour, bool asAMPM, bool pm)
	{
		uint8_t h = 0;
		h = TO_BCD(hour);
		if (asAMPM) {
			h |= 0b0100'0000; // flag: am/pm bit 6
			if (pm) {
				h |= 0b0010'0000; // bit 5 ampm
			}
		}
		return h;
	}

	bool updateAlarm1(uint8_t (&buf)[4])
	{
		// 0x07 - alarm1
		if (!TInterfaceClient::memWrite(ALARM1_START_REGISTER, buf, sizeof(buf))) {
			return false;
		}
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return false;
		uint8_t controlRegister = reg | 0b0000'0101; // INT & A1IE
		if (!setRegister(CONTROL_REGISTER, controlRegister))
			return false;
		// Bit 0: Alarm 1 Flag (A1F). A logic 1 in the alarm 1 flag
		// bit indicates that the time matched the alarm 1 regis-
		// ters. If the A1IE bit is logic 1 and the INTCN bit is set to
		// logic 1, the INT/SQW pin is also asserted. A1F is
		// cleared when written to logic 0. This bit can only be
		// written to logic 0. Attempting to write to logic 1 leaves
		// the value unchanged. (see page 14)

		// Therefore: Bit 0 (Alarm1) or bit 1 (Alarm2) must be cleared when
		// handled or want to handle next signal
		if (!getRegister(STATUS_REGISTER, reg))
			return false;
		uint8_t statusRegister = reg & 0b1111'1110; // clear Bit 0 for Alarm 1
		if (!setRegister(STATUS_REGISTER, statusRegister)) {
			return false;
		}
		return true;
	}

	bool updateAlarm2(uint8_t (&buf)[3])
	{
		// 0x07 - alarm1
		if (!TInterfaceClient::memWrite(ALARM2_START_REGISTER, buf, sizeof(buf))) {
			return false;
		}
		uint8_t reg = 0;
		if (!getRegister(CONTROL_REGISTER, reg))
			return false;
		uint8_t controlRegister = reg | 0b0000'0110; // set INT and A2IE
		if (!setRegister(CONTROL_REGISTER, controlRegister))
			return false;
		if (!getRegister(STATUS_REGISTER, reg))
			return false;
		uint8_t statusRegister = reg & 0b1111'1101; // clear Bit 1 for Alarm 2
		if (!setRegister(STATUS_REGISTER, statusRegister)) {
			return false;
		}
		return true;
	}

	enum : uint8_t {
		SECONDS_REGISTER = 0x00,
		MINUTE_REGISTER = 0x01,
		HOUR_REGISTER = 0x02,
		DAYOFWEEK_REGISTER = 0x03,
		DAYOFMONTH_REGISTER = 0x04,
		MONTH_REGISTER = 0x05,
		YEAR_REGISTER = 0x06,
		ALARM1_START_REGISTER = 0x07,
		ALARM2_START_REGISTER = 0x0B,
		CONTROL_REGISTER = 0x0E,
		STATUS_REGISTER = 0x0F,
		AGING_REGISTER = 0x10,
		TEMPERATURE_REGISTER = 0x11,
	};
	bool getRegister(uint8_t regAddr, uint8_t &outValue)
	{
		outValue = 0;
		if (!TInterfaceClient::memRead(regAddr, &outValue, sizeof(outValue))) {
			return false;
		}
		return true;
	}

	bool setRegister(uint8_t regAddr, uint8_t value)
	{
		if (!TInterfaceClient::memWrite(regAddr, &value, sizeof(value))) {
			return false;
		}
		return true;
	}
};

using DS3231 = DS3231Impl<I2CClientPolling>;

#endif // _DS3231_H_INCLUDED_
/*
Example

Declare a variable for clock:

I2CPollingModeMaster i2cInstance(&hi2c1);
DS3231 ds(&i2cInstance, DS3231::DS3231_I2CAddress);

Then simple use methods:
auto t = ds.dateTime();
// etc

Alarm:
Select any free pin and connect to SQW of DS3231.
Enable interrupts (in Cube: select for example PA12, then left click and select GPIO_EXTI12)
Mode: External Interrupt Mode with Falling edge trigger detection
Pull down/up: Pull-up

// create flag of interrupt:
bool clockInterrupt = false;

// implement callback function:
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
		case RTC_INT_Pin:
			// no other actions, people say, for example, in Arduino versions,
			// many actions in such handlers can freeze it.
			clockInterrupt = true;
			break;
		default:
			break;
	}
}

and handle "clockInterrupt" in main loop
*/

