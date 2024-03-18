# mclib

A library of various classes for microcontrollers.

These classes are not platform-specific: they use abstractions as dependency injections, e.g. `I2C` class.
If your platform is not present simple rewrite these classes (I2C, Delay etc).

## How to use

1. Define the platform macro:
* PLATFORM_STM32_HAL for STM32 if you use CubeHAL
2. Include the header file of the required class after platform-specific headers
3. Declare an instance of the I2C class appropriate for your platform.
```C++
// your code:
#include "stm32f4xx_hal.h" // for example STM32 & HAL for F4xx board

// define macro here (or as a compiler option if you are using multiple source files):
#define PLATFORM_STM32_HAL

// Include required headers (after "stm32f4xx_hal.h"), 
// for example for GY302 board with BH1750 sensor
#include <sensors/bh1750.h>

// ...more code...

// Declare I2C instance after initialization of I2C in your platform
I2C i2cInstance(&hi2c1);
GY302 gy302(&i2cInstance);

```

## Note for CubeIDE

Do not try rename `main.c` to `main.cpp`, this causes no effects.
CubeIDE currently unable to generate the `main.cpp` file, only `main.c`.

Therefore:

1. Right click your project and select "Convert to C++"
2. Right click `main.c` and select "Properties -> C/C++ Build -> Settings"
3. Select "MCU GCC Compiler" and type `g++` command instead of `gcc`
4. Then go to C/C++ General, select "Language mappings" and set "Default" to "GNU C++", otherwise you will get syntax errors in text editor

