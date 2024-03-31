# STM32 CubeHAL example for BH1750

## How to connect BH1750 sensor to STM32 board

1. Take NUCLEO, Blue pill or other board, in this example NUCLEO-F411RE is used.
2. Connect STM32 board to USB with ST-LINK or USB to TTL.
3. Create a project in the CubeIDE. Add to include path the `mclib` directory.
Change language mapping ang compiler of `main.c` to `g++`.
4. Enable I2C1 and USART2 interfaces
5. Connect pins to BH1750 board, for example GY-302 board may be used. The pull-up 10kOhm
resistors are present on this board and other resistors are not needed.

For example for NUCLEO-F411RE: 

* GND - 20 Left
* 3.3V - 16 Left
* PB7 (SDA) - 22 Left
* PB6 (SCL) - 17 Right
* GND (ADDR) - 22 Left

We connect ADDR to GND because ADDR pin can not be float: either GND, either VDD
to select I2C address.

Write the minimal working code in `main.c`:

```C++
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define PLATFORM_STM32_HAL
#include <sensors/BH1750.h>
#include <xuart.h>
/* USER CODE END Includes */
.............................
/* USER CODE BEGIN PV */
I2CPollingModeMaster i2c(&hi2c1);
BH1750 gy302(&i2c);

UARTLogger logger(&huart2);
/* USER CODE END PV */

int main(void)
{
......... generated code is here........
    /* USER CODE BEGIN 2 */
    logger.writeLine("Starting sensor...");
    if (!gy302.init()) {
        logger.writeLine("Unable to initialize BH1750, error: %d", (uint8_t) gy302.error());
        while (1);
    }
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if (gy302.ready()) {
            float lux = gy302.value();
            logger.writeLine("f=%d.%02d", (int) lux, ((int) (lux * 100)) % 100);
        }
    }
    /* USER CODE END 3 */
}
```

<img src="https://github.com/edarichev/mclib/blob/master/examples/STM32_HAL/STM32F411RET6_BH1750/direct_connection_bh1750.jpg" />
