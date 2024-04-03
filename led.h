/**
 * On-board LED
 * 
 * @author Evgeny Darichev
 * @date 03.04.2024
 */

#ifndef _LED_H_INCLUDED_
#define _LED_H_INCLUDED_

#include <delay.h>

class Led
{
#if defined(PLATFORM_STM32_HAL)
protected:
    GPIO_TypeDef* _port = nullptr;
    uint16_t _pin = (uint16_t)-1;
public:
    Led(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
        : _port(GPIOx), _pin(GPIO_Pin)
    {
        
    }

    inline void on()
    {
        HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_SET);
    }
    
    inline void off()
    {
        HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET);
    }
#else
#error "Please specify target platform"
#endif
    void blink(int8_t times)
    {
        for (uint8_t i = 0; i < times; ++i) {
            on();
            Delay::wait(700);
            off();
            Delay::wait(300);
        }
    }
};

#endif // _LED_H_INCLUDED_
