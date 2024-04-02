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

PA11: 
Select "GPIO_EXTI11"
GPIO Mode: External Interrupt Mode with Falling edge trigger detection
GPIO Pull-up/pul-down: Pull-up
Label: RTC_32KHZ

PA12:
Select "GPIO_EXTI12"
GPIO Mode: External Interrupt Mode with Falling edge trigger detection
GPIO Pull-up/pul-down: Pull-up
Label: RTC_INT

Enable System->NVIC->EXTI line [15:10] interrupts