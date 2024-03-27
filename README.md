# mclib

An experimental library of various classes for microcontrollers.

These device classes are not platform-specific: they use abstractions, e.g. `I2Cxxx` class.
If your platform is not present you can use most of it: simple rewrite some specific methods of these classes.

## What's here?

|Image|Device|Header|Class|
------|------|------|-----|
| <img src="https://github.com/edarichev/mclib/blob/master/images/bh1750.png" alt="BH1750 optical sensor" width="64" height="64" /> | BH1750 optical sensor | `bh1750.h` | `BH1750` |
| <img src="https://github.com/edarichev/mclib/blob/master/images/bmp280.png" alt="BMP280 temperature and pressure sensor" width="64" height="64" /> | BMP280 temperature and pressure sensor | `bmp280.h` | `BMP280` |
| <img src="https://github.com/edarichev/mclib/blob/master/images/lcd2004.png" alt="LCD2004 display" width="64" height="64" /> | LCD2004/1602 displays | `lcd1602.h` | `LCDTextDisplay` |



## How to use

1. Define a macro for the platform you need:
* `PLATFORM_STM32_HAL` for STM32 if you use CubeHAL (currently only this platform is supported)
2. Include the header file of the required class after platform-specific headers
3. Declare an instance of the I2C/SPI etc. protocol class appropriate for your platform.
```C++
// your existing code:
#include "stm32f4xx_hal.h" // for example STM32 & HAL for F4xx board

// 1. define macro here (or as a compiler option if you are using multiple source files):
#define PLATFORM_STM32_HAL

// 2. Include required headers (after "stm32f4xx_hal.h"), 
// This example for GY302 board with BH1750 sensor
#include <sensors/bh1750.h>

// ...more code...

// 3. Declare I2C instance after initialization of I2C in your platform
I2C i2cInstance(&hi2c1);
BH1750 gy302(&i2cInstance);
```

## Note for CubeIDE

Do not try rename `main.c` to `main.cpp`, this causes no effects.
CubeIDE currently unable to generate the `main.cpp` file, only `main.c`.

Therefore:

1. Right click your project and select "Convert to C++"
2. Right click `main.c` and select "Properties -> C/C++ Build -> Settings"
3. Select "MCU GCC Compiler" and type `g++` command instead of `gcc`
4. Then go to C/C++ General, select "Language mappings" and set "Default" to "GNU C++", otherwise you will get syntax errors in text editor

