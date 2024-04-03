/**
 * Step motor 28BYJ-48
 *
 * @author Evgeny Darichev
 * @date 03.04.2024
 */

#ifndef _28BYJ48_H_INCLUDED_
#define _28BYJ48_H_INCLUDED_

#include <initializer_list>
#include <array>

class StepMotor28BYJ48
{
    // red, orange, yellow, blue
    static const constexpr uint8_t PIN_COUNT = 4;
    // полношаговый режим – 4 ступени импульсов на 1 шаг;
//      A  B  C  D == 1,2,3,4
//    { 1, 0, 1, 0 }, // 1 такт
//    { 0, 1, 1, 0 }, // 2 такт
//    { 0, 1, 0, 1 }, // 3 такт
//    { 1, 0, 0, 1 }, // 4 такт
    // последовательность выше закодируем в группы по 4 бита от младших к старшим
    //                                                                                   ABCD
    static const constexpr uint32_t Step4Sequence = 0b0000'0000'0000'0000'1001'0101'0110'1010;
    // полушаговый режим – 8 ступеней импульсов на 1 шаг.
    static const constexpr uint32_t Step8Sequence = 0b1001'0001'0101'0100'0110'0010'1010'1000;
    uint8_t _step = 0;
    uint8_t _tCount = 4; // число тактов: 4 или 8
    uint32_t _currentPattern = 0;
    bool _right = true;

#if defined(PLATFORM_STM32_HAL)
    std::array<GPIO_TypeDef*, PIN_COUNT> _port;
    std::array<uint16_t, PIN_COUNT> _pin;
private:
    void setPin(uint8_t pinNumber, bool pinOn)
    {
        HAL_GPIO_WritePin(_port[pinNumber], _pin[pinNumber],
                pinOn ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
public:
    StepMotor28BYJ48(const std::array<GPIO_TypeDef*, PIN_COUNT> &ports,
            const std::array<uint16_t, PIN_COUNT> &pins)
        : _port(ports), _pin(pins)
    {

    }
#endif

    // TODO: можно при переключении направления отыскивать
    // такой же паттерн и вычислять шаг из него

    void beginRotateRight4()
    {
        _step = 0;
        _tCount = 4;
        _currentPattern = Step4Sequence;
        _right = true;
    }

    void beginRotateRight8()
    {
        _step = 0;
        _tCount = 8;
        _currentPattern = Step8Sequence;
        _right = true;
    }

    void beginRotateLeft4()
    {
        _step = 0;
        _tCount = 4;
        _currentPattern = Step4Sequence;
        _right = false;
    }

    void beginRotateLeft8()
    {
        _step = 0;
        _tCount = 8;
        _currentPattern = Step8Sequence;
        _right = false;
    }

    void step()
    {
        _step %= _tCount;
        uint32_t pinState = _currentPattern;
        if (!_right) {
            uint32_t d = _currentPattern == Step4Sequence ? 16 - 4 : 32 - 4;
            pinState >>= (d - _step * 4);
        } else
            pinState >>= _step * 4;
        // группа в 4 бита теперь слева
        for (uint8_t i = 0; i < PIN_COUNT; ++i) {
            uint32_t p = pinState;
            p >>= (3 - i); // нужный пин крайний слева
            p &= 1;
            setPin(i, p);
        }
        _step++;
    }
};

#endif // _28BYJ48_H_INCLUDED_
