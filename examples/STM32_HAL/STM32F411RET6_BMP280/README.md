# STM32 CubeHAL I2C example for BMP280

## How to connect BMP280 sensor to STM32 board 

In this example we use NUCLEO-411RE and GY-BM ME/PM 280 with I2C1 interface. 

| GY-BM ME/PM 280 | NUCLEO-411RE |
|-----------------|--------------|
| VCC             | +3.3V 16 Left|
| GND             | GND 20 Left  |
| SCL             | PB6 17 Right |
| SDA             | PB7 21 Left  |
| CS              | +3.3V or VDD (5 Left) to select I2C interface |
| SDO             | GND to select 0x76 (or VCC for 0x77) I2C adress |

<img src="https://github.com/edarichev/mclib/blob/master/examples/STM32_HAL/STM32F411RET6_BMP280/connect_i2c_bmp280.jpg" />
