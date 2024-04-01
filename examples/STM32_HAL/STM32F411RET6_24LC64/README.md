# STM32 CubeHAL example for 24LC64 EEPROM

## How to connect 24LC64 EEPROM to STM32 board

For example for NUCLEO-F411RE and I2C1 interface: 

| 24LC64 | NUCLEO-F411RE | Comment |
|--------|---------------|---------|
| VCC    | 3.3V, 16 Left | |
| SDA    | PB7, 21 Left  | |
| SCL    | PB6, 17 Right | |
| A0     | GND, 20 Left  | Start address is 0x50 with A0=A1=A2=GND |
| A1     | GND, 20 Left  | |
| A2     | GND, 20 Left  | |
| Vss    | GND, 20 Left  | |
| WP     | GND, 20 Left  | Write protect, connect to VCC to disable writing and to GND to enable. |

<img src="https://github.com/edarichev/mclib/blob/master/examples/STM32_HAL/STM32F411RET6_24LC64/24lc64_pdip.png" />

<img src="https://github.com/edarichev/mclib/blob/master/examples/STM32_HAL/STM32F411RET6_24LC64/24lc64_pdip.png" />
