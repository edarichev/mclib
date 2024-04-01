# mclib

An experimental library of various classes for microcontrollers.

These device classes are not platform-specific: they use abstractions, e.g. `I2Cxxx` class.
If your platform is not present you can use most of it: simple rewrite some specific methods of these classes.

## What's here?

| <div style="width:100px">&nbsp;&nbsp;&nbsp;Image&nbsp;&nbsp;&nbsp;</div> |Device|Header|Class|Example|
|-------|------|------|-----|-------|
| <img src="https://github.com/edarichev/mclib/blob/master/images/bh1750.png" alt="BH1750 optical sensor" width="64" height="64" /> | BH1750 optical sensor | `BH1750.h` | `BH1750` class |<a href="https://github.com/edarichev/mclib/tree/master/examples/STM32_HAL/STM32F411RET6_BH1750">HAL</a>|
| <img src="https://github.com/edarichev/mclib/blob/master/images/bmp280.png" alt="BMP280 temperature and pressure sensor" width="64" height="64" /> | BMP280 temperature and pressure sensor | `BMP280.h` | `BMP280` class |<a href="https://github.com/edarichev/mclib/tree/master/examples/STM32_HAL/STM32F411RET6_BMP280">HAL</a>|
| <img src="https://github.com/edarichev/mclib/blob/master/images/lcd2004.png" alt="LCD2004 display" width="64" height="64" /> | LCD2004/1602 displays | `display/LCD1602.h` | `LCDTextDisplay`, `LCD1602`, `LCD2004` |<a href="https://github.com/edarichev/mclib/tree/master/examples/STM32_HAL/STM32F411RET6_LCD2004_I2C">HAL I2C</a>|
| <img src="https://github.com/edarichev/mclib/blob/master/images/24lc64.png" alt="24LC64" width="64" height="64" /> | 24LC64 EEPROM | `eeprom/24LC64.h` | `memtest_24LC64` function for memory test ||
| <img src="https://github.com/edarichev/mclib/blob/master/images/DS3231.png" alt="DS3231" width="64" height="64" /> | DS3231 real time clock | `clock/DS3231.h` | `DS3231` class ||


## How to use

All classes are used according to an uniform scheme:
1. Define a macro for the platform you need:
* `PLATFORM_STM32_HAL` for STM32 if you use CubeHAL (currently only this platform is supported)
2. Include the header file of the required class after platform-specific headers
3. Declare an instance of the I2C/SPI etc. protocol class appropriate for your platform.

For example:

```C++
// your existing code:
#include "stm32f4xx_hal.h" // for example STM32 & HAL for F4xx board

// 1. define macro here (or as a compiler option if you are using multiple source files):
#define PLATFORM_STM32_HAL

// 2. Include required headers (after "stm32f4xx_hal.h"), 
// This example for BH1750 sensor
#define PLATFORM_STM32_HAL
#include <sensors/BH1750.h>

// 3. Declare I2C instance after initialization of I2C in your platform
I2CPollingModeMaster i2c(&hi2c1);
BH1750 gy302(&i2c);
// other code
```

## UART Logging

For logging via the UART interface, there is the UARTLogger class. 
This class also contains the `xassert` method to create simple unit tests.

To view UART output you can use `kermit` or another tool.
```
sudo apt install kermit
```

Add the `.kermrc` file into your home directory with contents:
```
set line /dev/ttyACM1
set speed 115200
set carrier-watch off
c
```

To view your COM-port device use this command:
```
ls /dev/
```
This may be `ttyACM0` or `ttyACM1`.

## Note for CubeIDE

Do not try rename `main.c` to `main.cpp`, this causes no effects.
CubeIDE currently unable to generate the `main.cpp` file, only `main.c`.

Therefore:

1. Right click your project and select "Convert to C++"
2. Right click `main.c` and select "Properties -> C/C++ Build -> Settings"
3. Select "MCU GCC Compiler" and type `g++` command instead of `gcc`
4. Then go to C/C++ General, select "Language mappings" and set "Default" to "GNU C++", otherwise you will get syntax errors in text editor

