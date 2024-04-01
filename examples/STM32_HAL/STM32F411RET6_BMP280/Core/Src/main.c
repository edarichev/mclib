/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define PLATFORM_STM32_HAL
#include <sensors/BMP280.h>
#include <xuart.h>
#include <i2cscanner.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
I2CPollingModeMaster i2c(&hi2c1);
UARTLogger logger(&huart2);

BMP280TemperatureOversampling t_osrs[] = {
        BMP280TemperatureOversampling::Skipped,
        BMP280TemperatureOversampling::UltraLowPower,
        BMP280TemperatureOversampling::LowPower,
        BMP280TemperatureOversampling::StandardResolution,
        BMP280TemperatureOversampling::HighResolution,
        BMP280TemperatureOversampling::UltraHighResolution, };
BMP280PressureOversampling p_osrs[] = { BMP280PressureOversampling::Skipped,
        BMP280PressureOversampling::UltraLowPower,
        BMP280PressureOversampling::LowPower,
        BMP280PressureOversampling::StandardResolution,
        BMP280PressureOversampling::HighResolution,
        BMP280PressureOversampling::UltraHighResolution, };
BMP280PowerMode power[] = { BMP280PowerMode::Sleep, BMP280PowerMode::Normal,
        BMP280PowerMode::Forced, BMP280PowerMode::Forced2, };
BMP280IIRFilterMode filter[] = { BMP280IIRFilterMode::FilterOff,
        BMP280IIRFilterMode::Filter2, BMP280IIRFilterMode::Filter4,
        BMP280IIRFilterMode::Filter8, BMP280IIRFilterMode::Filter16 };
const char *filterName[] = { "FilterOff", "Filter2", "Filter4", "Filter8",
        "Filter16" };
const char *powerName[] = { "Sleep", "Normal", "Forced", "Forced2", };
// both for T and P
const char *osName[] = { "Skipped", "UltraLowPower", "LowPower",
        "StandardResolution", "HighResolution", "UltraHighResolution", };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
template<class TBMP280>
void printValue(TBMP280 &bmp)
{
    if (bmp.ready()) {
        int32_t t = 0;
        uint32_t p = 0;
        if (!bmp.readData(t, p)) {
            logger.writeLine("Unable to read P, T, error: %d",
                    (uint8_t) bmp.error());
        }
        else {
            float v = BMP280::mmhg<float>(p);
            logger.write("T=%d, P=%d.%02d mm hg, %d pa\r\n", (uint32_t) t,
                    (int) v, (int) (100 * v) % 100, p);
        }
    }
    else {
        logger.writeLine("Sensor not ready, error: %d", (uint8_t) bmp.error());
    }
}

template<class TBMP280>
void printInMode(TBMP280 &bmp, BMP280TemperatureOversampling t,
        BMP280PressureOversampling p, BMP280PowerMode m, BMP280IIRFilterMode f,
        const char *tosName, const char *posName, const char *modeName,
        const char *filterName)
{
    char buf[180];
    sprintf(buf, "T: %s, P: %s, M: %s, F: %s\r\n", tosName, posName, modeName,
            filterName);
    logger.write(buf);
    int n = 5; // 5 measurements
    int i = 0;
    if (!bmp.setTemperatureOversampling(t))
        logger.writeLine("Unable to set t_osrs, error: %d", (uint8_t) bmp.error());
    if (!bmp.setPressureOversampling(p))
        logger.writeLine("Unable to set p_osrs, error: %d", (uint8_t) bmp.error());
    if (!bmp.setPowerMode(m))
        logger.writeLine("Unable to set power mode, error: %d", (uint8_t) bmp.error());
    if (!bmp.setIIRFilter(f))
        logger.writeLine("Unable to set IIR filter, error: %d", (uint8_t) bmp.error());
    for (i = 0; i < n; i++) {
        Delay::wait(200); // wait after setting parameters above before getting values
        printValue(bmp);
    }
    Delay::wait(2000);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    // enumerate I2C devices:
    uint8_t ic = I2CScanner::enumerate(&hi2c1, [](auto addr) {
        logger.writeLine("I2C device found at address %#2X", addr);
    });
    logger.writeLine("%d I2C devices found", ic);

    logger.writeLine("Creating sensor instance");
    BMP280 bmp(&i2c);
    logger.writeLine("Initializing...");
    if (!bmp.init()) {
        logger.writeLine("Unable to initialize the sensor, error: %d",
                (uint8_t) bmp.error());
    }
    uint8_t cd = bmp.chipId();
    logger.writeLine("chipid=%#X", cd);
    if (cd == BMP280::INVALID_CHIP_ID)
        logger.writeLine("error=%d", (uint8_t) bmp.error());

    logger.writeLine("Setting F4 registry");
    if (!bmp.setOversamplingRegistry(
            BMP280TemperatureOversampling::StandardResolution,
            BMP280PressureOversampling::StandardResolution,
            BMP280PowerMode::Normal)) {
        logger.writeLine("Unable to set F4 register, error: %d",
                (uint8_t) bmp.error());
    }
    logger.writeLine("Setting t_standby");
    if (!bmp.setStandbyTime(BMP280StandbyTime::Standby_250)) {
        logger.writeLine("Unable to set t_standby, error: %d", (uint8_t) bmp.error());
    }
    logger.writeLine("Setting IRRFilter");
    if (!bmp.setIIRFilter(BMP280IIRFilterMode::FilterOff)) {
        logger.writeLine("Unable to set IIR filter, error: %d", (uint8_t) bmp.error());
    }
    logger.writeLine("First run:");
    Delay::wait(500);
    // print current settings
    printInMode(bmp, t_osrs[3], p_osrs[3], power[1], filter[0], osName[3],
            osName[3], powerName[1], filterName[0]);
    logger.writeLine("Starting main loop");
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // Set all possible values
        logger.writeLine("*** STARTING TEST SERIES ***");
        // 4 filer * 6 t_osrs * 6 p_osrs * 4 power = 720 variants
        for (uint8_t t = 0;
                t < sizeof(t_osrs) / sizeof(BMP280TemperatureOversampling);
                t++) {
            for (uint8_t p = 0;
                    p < sizeof(p_osrs) / sizeof(BMP280PressureOversampling);
                    p++) {
                for (uint8_t m = 0; m < sizeof(power) / sizeof(BMP280PowerMode);
                        m++) {
                    for (uint8_t f = 0;
                            f < sizeof(filter) / sizeof(BMP280IIRFilterMode);
                            f++) {
                        printInMode(bmp, t_osrs[t], p_osrs[p], power[m], filter[f], osName[t], osName[p],
                                powerName[m], filterName[f]);
                    }
                }
            }
        }
        logger.writeLine("*** END OF TEST SERIES ***");
        Delay::wait(2000);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 100;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
