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
	uint8_t ampm = 0;
	// time in 12-hour format
	uint8_t asAMPM = 0;

	void toAMPM()
	{
		if (asAMPM)
			return; // already AMPM
		asAMPM = 1;
		ampm = hour > 12 ? 1 : 0;
		if (hour > 12)
			hour -= 12;
	}

	void to24()
	{
		if (!asAMPM)
			return;
		asAMPM = 0;
		if (ampm)
			hour += 12;
		ampm = 0;
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
	};
public:
	template <typename U = TInterfaceClient>
	DS3231Impl(typename U::InterfacePtrType i2c, uint8_t addr)
		: TInterfaceClient(i2c, (uint16_t) (addr << 1))
	{
		
	}
	
	DS3231DateTime readTime()
	{
		DS3231DateTime t;
		uint8_t buf[7]; // 7 bytes for alarm and clock data
		if (!TInterfaceClient::memRead(0x00, buf, sizeof(buf))) {
			return t;
		}
		
		t.second = FROM_BCD(buf[0]);
		t.minute = FROM_BCD(buf[1]);
		uint8_t hb = buf[2];
		t.asAMPM = (hb & 0b0100'0000) ? 1 : 0;
		t.hour = FROM_BCD(0b0000'1111 & hb);
		uint8_t hadd = 0b0001'0000 & hb ? 10 : 0; // +10?
		t.ampm = (t.asAMPM && (0b0010'0000 & hb)) ? 1 : 0;
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

	bool writeTime(const DS3231DateTime &t)
	{
		uint8_t buf[7]; // 7 bytes for alarm and clock data
		buf[0] = TO_BCD(t.second);
		buf[1] = TO_BCD(t.minute);
		uint8_t h = 0;
		h = TO_BCD(t.hour);
		if (t.asAMPM) {
			h |= 0b0100'0000; // flag: am/pm bit 6
			if (t.ampm) {
				h |= 0b0010'0000; // bit 5 ampm
			}
		}
		buf[2] = h;
		buf[3] = t.dayOfWeek;
		buf[4] = TO_BCD(t.day);
		buf[5] = TO_BCD(t.month) | (t.century << 7);
		buf[6] = TO_BCD(t.year);
		if (!TInterfaceClient::memWrite(0x00, buf, sizeof(buf))) {
			return false;
		}
		return true;
	}
};

using DS3231 = DS3231Impl<I2CClientPolling>;

#endif // _DS3231_H_INCLUDED_
