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
#include <sensors/BH1750.h>
#include <xuart.h>
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
BH1750 gy302(&i2c);

UARTLogger logger(&huart2);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */
    logger.writeLine("Starting sensor...");
    if (!gy302.init()) {
        logger.writeLine("Unable to initialize BH1750, error: %d", (uint8_t) gy302.error());
    }

    // the next code is not necessary in use, but this is a full test:
    logger.xassert(gy302.powerDown(), "powerDown failed, error: %d", (uint8_t) gy302.error());
    logger.xassert(gy302.powerOn(), "powerOn failed, error: %d", (uint8_t) gy302.error());
    logger.xassert(gy302.reset(), "reset failed, error: %d", (uint8_t) gy302.error());
    logger.xassert(gy302.setMTReg(56), "setMTReg failed, error: %d", (uint8_t) gy302.error());
    logger.xassert(gy302.setPowerState(BH1750PowerState::PowerDown), "setPowerState:powerDown failed, error: %d", (uint8_t) gy302.error());
    logger.xassert(gy302.setPowerState(BH1750PowerState::PowerOn), "setPowerState:powerOn failed, error: %d", (uint8_t) gy302.error());
    logger.xassert(gy302.setPowerState(BH1750PowerState::Reset), "setPowerState:reset failed, error: %d", (uint8_t) gy302.error());
    logger.xassert(gy302.setResolution(BH1750ResolutionMode::ContinuouslyLow), "setResolution failed, error: %d", (uint8_t) gy302.error());
    float v1 = gy302.value();
    logger.xassert(!std::isnan(v1), "Invalid value v1, error: %d", (uint8_t) gy302.error());
    logger.xassert(gy302.setResolution(BH1750ResolutionMode::OneTimeLow), "setResolution:OneTimeLow failed, error: %d", (uint8_t) gy302.error());
    float v2 = gy302.value();
    logger.xassert(!std::isnan(v2), "Invalid value v2, error: %d", (uint8_t) gy302.error());
    float e = 20; // possible deviation
    logger.xassert(std::abs(v2 - v1) < e, "Value v2 != v1, error: %d", (uint8_t) gy302.error());
    gy302.reset(); // reset the device after every measuring in OneTimeXXX modes

    logger.xassert(gy302.setResolution(BH1750ResolutionMode::OneTimeHigh1), "setResolution:OneTimeHigh1 failed, error: %d", (uint8_t) gy302.error());
    float v3 = gy302.value();
    logger.xassert(!std::isnan(v3), "Invalid value v3, error: %d", (uint8_t) gy302.error());
    logger.xassert(std::abs(v3 - v1) < e, "Value v3 != v1, error: %d", (uint8_t) gy302.error());
    gy302.reset();

    logger.xassert(gy302.setResolution(BH1750ResolutionMode::OneTimeHigh2), "setResolution:OneTimeHigh2 failed, error: %d", (uint8_t) gy302.error());
    float v4 = gy302.value();
    logger.xassert(!std::isnan(v4), "Invalid value v4, error: %d", (uint8_t) gy302.error());
    logger.xassert(std::abs(v4 - v1) < e, "Value v4 != v1, error: %d", (uint8_t) gy302.error());
    gy302.reset();

    logger.xassert(gy302.setResolution(BH1750ResolutionMode::ContinuouslyHigh1), "setResolution:ContinuouslyHigh1 failed, error: %d", (uint8_t) gy302.error());
    logger.xassert(gy302.setResolution(BH1750ResolutionMode::ContinuouslyHigh2), "setResolution:ContinuouslyHigh2 failed, error: %d", (uint8_t) gy302.error());

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if (gy302.ready()) {
            float lux = gy302.value();
            logger.writeLine("f=%d.%02d; error=%d", (int) lux, ((int) (lux * 100)) % 100, (uint8_t)gy302.error());
        }
        Delay::wait(1000);
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
