#ifndef _LCD1602_H_INCLUDED_
#define _LCD1602_H_INCLUDED_

#include "../delay.h"
#include "../i2cclient.h"

enum class LCDTextDisplayControlBit
{
	E = 0b0000'0100,
	RW = 0b00000010,
	RS = 0b00000001,
};

enum class LCDTextDisplayCommand : uint8_t
{
	//**************R:*SW 7654 3210
	ClearDisplay   = 0b00'0000'0001,
	ReturnHome     = 0b00'0000'0010,
	EntryModeSet   = 0b00'0000'0100,
	DisplayControl = 0b00'0000'1000,
	CursorShift    = 0b00'0001'0000,
	FunctiionSet   = 0b00'0010'0000,
	SetCGRAMAddr   = 0b00'0100'0000,
	SetDDRAMAddr   = 0b00'1000'0000,
};

enum class LCDTextDisplayEntryMode : uint8_t
{
	Right = 0,
	Left = 0b00'0000'0010,
	DisplayShiftIncrement = 0b00'0000'0001,
	DisplayShiftDecrement = 0,
};

enum LCDTextDisplayControl : uint8_t
{
	// flags for display on/off control
	DisplayOn = 0b00'0000'0100,
	DisplayOff = 0,
	CursorOn =  0b00'0000'0010,
	CursorOff = 0,
	BlinkOn =   0b00'0000'0001,
	BlinkOff = 0,
};

enum class LCDTextDisplayFunctionSet
{
	Mode8Bit = 0b00'0001'0000,
	Mode4Bit = 0,
	Line2 =    0b00'0000'1000,
	Line1 = 0,
	Size5x10 = 0b00'0000'0100,
	Size5x8 = 0,
};

enum class LCDTextDisplayBacklight
{
	Backlight = 0b00'0000'1000,
	NoBacklight = 0,
};

enum class LCDTextDisplaySendFlag
{
	Data = 1,
	Command = 0,
};

template <uint8_t ROWS, uint8_t COLUMNS, class TInterfaceClient = I2CPollingClient>
class LCDTextDisplay : public TInterfaceClient
{
protected:
	using BaseClass = I2CClientImpl<I2CPollingModeMaster>;
	uint8_t _rows = 0;
	uint8_t _columns = 0;
    uint8_t _displaycontrol;
    uint8_t _displaymode;
    uint8_t _fontSize = (uint8_t)LCDTextDisplayFunctionSet::Size5x8;
    LCDTextDisplayBacklight _backlight = LCDTextDisplayBacklight::NoBacklight;
    LCDTextDisplayFunctionSet _lineMode = LCDTextDisplayFunctionSet::Line1;

public:
    
	LCDTextDisplay(typename TInterfaceClient::InterfacePtrType i2c, uint8_t addr, uint8_t rows = ROWS, uint8_t columns = COLUMNS)
			: TInterfaceClient(i2c, (uint16_t) (addr << 1)), _rows(rows), _columns(columns)
	{
		_displaycontrol = (uint8_t)LCDTextDisplayControl::CursorOff | (uint8_t)LCDTextDisplayControl::BlinkOff;
		_displaymode = (uint8_t)LCDTextDisplayEntryMode::Left | (uint8_t)LCDTextDisplayEntryMode::DisplayShiftDecrement;
	}

	void init(bool useFont5x8 = true)
	{
		_fontSize = useFont5x8 ? (uint8_t)LCDTextDisplayFunctionSet::Size5x8 : (uint8_t)LCDTextDisplayFunctionSet::Size5x10;
		// for delay values see page 12
		delay(50);  // INIT: wait for >40ms

	  	// see page 13, 4-bit interface requires 8 bit init
		// function set, interface is 8 bit length #1
		command((uint8_t)LCDTextDisplayCommand::FunctiionSet | (uint8_t)LCDTextDisplayFunctionSet::Mode8Bit);
	  	delay(5);
	  	// function set, interface is 8 bit length #2
		command((uint8_t)LCDTextDisplayCommand::FunctiionSet | (uint8_t)LCDTextDisplayFunctionSet::Mode8Bit);
	  	delay(1); // > 100 us -> 1 ms
	  	// function set, interface is 8 bit length #3
	  	command((uint8_t)LCDTextDisplayCommand::FunctiionSet |
	  			(uint8_t)LCDTextDisplayFunctionSet::Mode8Bit);
	  	// now set the interface to 4 bit length (but now in 8-bit interface)
		command((uint8_t)LCDTextDisplayCommand::FunctiionSet |
				(uint8_t)LCDTextDisplayFunctionSet::Mode4Bit);
		// and now 4 bit interface activated, continue setup the device (see sequence on page 13)
		// specify the number of the display lines and character font
		command((uint8_t)LCDTextDisplayCommand::FunctiionSet |
				(uint8_t)LCDTextDisplayFunctionSet::Mode4Bit |
				_fontSize |
				(uint8_t)_lineMode);
		// the number of the display lines and character font can not be changed afterwards
		// display off
		command((uint8_t)LCDTextDisplayControl::DisplayOff |
				(uint8_t)LCDTextDisplayControl::CursorOff |
				(uint8_t)LCDTextDisplayControl::BlinkOff);
		// display clear
		command((uint8_t)LCDTextDisplayCommand::ClearDisplay);
		delay(2);
		// initialization ends

		// entry mode set

		command((uint8_t)LCDTextDisplayCommand::EntryModeSet | _displaymode);
		command((uint8_t)LCDTextDisplayCommand::DisplayControl | (uint8_t)LCDTextDisplayControl::DisplayOn | (uint8_t)_displaycontrol);
	}

	void home()
	{
		command((uint8_t)LCDTextDisplayCommand::ReturnHome);
		delay(2);
	}

	void clear()
	{
		// очистка дисплея
		command((uint8_t)LCDTextDisplayCommand::ClearDisplay);
		delay(2);
	}

	void backlight(bool onOff)
	{
		_backlight = onOff ? LCDTextDisplayBacklight::Backlight :
				LCDTextDisplayBacklight::NoBacklight;
		command(0);
	}

	void blink(bool onOff)
	{
		if (onOff)
			_displaycontrol |= (uint8_t)LCDTextDisplayControl::BlinkOn;
		else
			_displaycontrol &= ~(uint8_t)LCDTextDisplayControl::BlinkOn;
		command((uint8_t)LCDTextDisplayCommand::DisplayControl |
				(uint8_t)LCDTextDisplayControl::DisplayOn | _displaycontrol);
	}

	void cursor(bool onOff)
	{
		if (onOff)
			_displaycontrol |= (uint8_t)LCDTextDisplayControl::CursorOn;
		else
			_displaycontrol &= ~(uint8_t)LCDTextDisplayControl::CursorOn;
		command((uint8_t)LCDTextDisplayCommand::DisplayControl |
				(uint8_t)LCDTextDisplayControl::DisplayOn | _displaycontrol);
	}

	void setCursor(uint8_t row, uint8_t col)
	{
		if (row >= _rows)
			return;
		int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };

		command((uint8_t)LCDTextDisplayCommand::SetDDRAMAddr | (col + row_offsets[row]));
	}

	void print(const char *str)
	{
		while(*str) {
			send((uint8_t)(*str), LCDTextDisplaySendFlag::Data);
	        str++;
	    }
	}

	uint8_t command(uint8_t command)
    {
        return send(command, LCDTextDisplaySendFlag::Command);
    }

	bool shiftDisplayRight()
	{
	    uint8_t command = (uint8_t)LCDTextDisplayEntryMode::Right;
	    return send(command, LCDTextDisplaySendFlag::Command);
	}

    bool shiftDisplayLeft()
    {
        uint8_t command = (uint8_t)LCDTextDisplayEntryMode::Left;
        return send(command, LCDTextDisplaySendFlag::Command);
    }

	bool send(uint8_t data, LCDTextDisplaySendFlag flags)
	{
		// We can use only 4 bits to data transfer
		// therefore we split one byte into 2 parts
		uint8_t up = data & 0xF0; // upper part
		uint8_t lo = (data << 4) & 0xF0; // lower part

		uint8_t buf[4];
		// msb [7654] - data, lsb[3210] - configuration
		uint8_t f = (uint8_t)flags | (uint8_t)_backlight;
		buf[0] = up | f | (uint8_t)LCDTextDisplayControlBit::E;
		// дублирование сигнала, на выводе Е в этот раз 0
		buf[1] = up | f;
		// всё то же самое, дёргаем второй раз за ногу битом E
		buf[2] = lo | f | (uint8_t)LCDTextDisplayControlBit::E;
		buf[3] = lo | f;

		return TInterfaceClient::write(buf, sizeof(buf));
	}

protected:
	void delay(uint32_t t)
	{
		Delay::wait(t);
	}
};

using LCD2004 = LCDTextDisplay<4, 20, I2CPollingClient>;
using LCD1602 = LCDTextDisplay<2, 16, I2CPollingClient>;

#endif // _LCD1602_H_INCLUDED_
