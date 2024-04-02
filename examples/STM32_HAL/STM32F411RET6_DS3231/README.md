# STM32 CubeHAL example for DS3231 real time clock

## How to connect DS3231 to STM32 board

For example for NUCLEO-F411RE and I2C1 interface: 

| DS3231 | NUCLEO-F411RE |
|--------|---------------|
| GND    | GND, 20 Left  |
| VCC    | 3.3V, 16 Left |
| SDA    | PB7, 21 Left  |
| SCL    | PB6, 17 Right |
| SQW    | PA12, 12 Right |
| 32K    | PA11, 14 Right |

PA11: <br/>
Select "GPIO_EXTI11"<br/>
GPIO Mode: External Interrupt Mode with Falling edge trigger detection<br/>
GPIO Pull-up/pull-down: Pull-up<br/>
Label: RTC_32KHZ

PA12:<br/>
Select "GPIO_EXTI12"<br/>
GPIO Mode: External Interrupt Mode with Falling edge trigger detection<br/>
GPIO Pull-up/pull-down: Pull-up<br/>
Label: RTC_INT<br/>

Enable System->NVIC->EXTI line [15:10] interrupts

<img src="https://github.com/edarichev/mclib/blob/master/examples/STM32_HAL/STM32F411RET6_DS3231/ds3231_maket.jpg" />

