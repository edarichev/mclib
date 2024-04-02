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
#include <cstring>
#include <xuart.h>
#include <i2cscanner.h>
#include <clock/DS3231.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// set to not 0 to run unit test
#define DS3231_FULL_TEST 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
I2CPollingModeMaster i2cInstance(&hi2c1);
DS3231 ds(&i2cInstance, DS3231::DS3231_I2CAddress);
bool clockInterrupt = false; // for alarm, pin PA12
uint32_t clock32 = 0;
char buf[100];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
    case RTC_INT_Pin:
        clockInterrupt = true;
        break;
    case RTC_32KHZ_Pin:
        clock32++;
        break;
    default:
        break;
    }
}

void setClockAlarm(DS3231 &ds)
{
    ds.setAlarm2MinuteMatch(0);
    ds.setAlarm1SecondMatch(18);
    ds.setAlarm2Enabled(false);
}

void printTime(DS3231 &ds, UARTLogger &logger)
{
    auto t = ds.dateTime();
    sprintf(buf,
            "%d c, %02d.%02d.%02d %02d:%02d:%02d, pm=%d, asAMPM=%d, wd=%d\r\n",
            t.century, t.day, t.month, t.year, t.hour, t.minute, t.second,
            t.pm, t.asAMPM, t.dayOfWeek);
    logger.write(buf);
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
    UARTLogger logger(&huart2);
    uint8_t ic = I2CScanner::enumerate(&hi2c1, [&](auto addr) {
        logger.writeLine("I2C device found at address %#2X", addr);
    });
    logger.writeLine("%d I2C devices found", ic);

#if DS3231_FULL_TEST
    // The complex test of DS3231 class

    /*** WARNING: the value of the DS3231 clock will be overwritten ***/

    logger.writeLine("** Set and check values test case ***");
    DS3231DateTime
    t { .second = 0, .minute = 44, .hour = 15, .dayOfWeek = 7, .day = 30,
            .month = 3, .year = 24, .century = 1, .pm = 0, .asAMPM = 0, };

    ds.setDateTime(t);
    Delay::wait(500);
    // second now must be 0..2 but not more than 5
    logger.xassert(ds.second() - t.second < 5, "Second not match");
    logger.xassert(ds.minute() == t.minute, "Minute not match");
    logger.xassert(ds.hour() == t.hour, "Hour not match");
    logger.xassert(ds.dayOfWeek() == t.dayOfWeek, "Day of week not match");
    logger.xassert(ds.day() == t.day, "Day not match");
    logger.xassert(ds.month() == t.month, "Month not match");
    logger.xassert(ds.year() == t.year, "Year not match");
    logger.xassert(ds.century() == t.century, "Century not match");
    logger.xassert(ds.asAMPM() == t.asAMPM, "asAMPM not match");
    logger.xassert(ds.pm() == t.pm, "pm not match");

    logger.writeLine("** Checking each function test case ***");
    // check each function
    //
    struct DS3231DateTime
        t2 { .second = 10, .minute = 14, .hour = 5, .dayOfWeek = 2, .day = 1,
                .month = 6, .year = 4, .century = 1, .pm = 1, .asAMPM = 1, };
    ds.setHour(t2.hour, t2.asAMPM, t2.pm);
    Delay::wait(200);
    logger.xassert(ds.hour() == t2.hour, "Hour not match");
    logger.xassert(ds.asAMPM() == t2.asAMPM, "asAMPM not match");
    logger.xassert(ds.pm() == t2.pm, "pm not match");
    ds.setSecond(t2.second);
    Delay::wait(200);
    logger.xassert(ds.second() - t2.second < 5, "Second not match");
    ds.setMinute(t2.minute);
    Delay::wait(200);
    logger.xassert(ds.minute() == t2.minute, "Minute not match");
    ds.setDayOfWeek(t2.dayOfWeek);
    Delay::wait(200);
    logger.xassert(ds.dayOfWeek() == t2.dayOfWeek, "Day of week not match");
    ds.setDay(t2.day);
    Delay::wait(200);
    logger.xassert(ds.day() == t2.day, "Day not match");
    ds.setMonth(t2.month);
    Delay::wait(200);
    logger.xassert(ds.month() == t2.month, "Month not match");
    ds.setYear(t2.year);
    Delay::wait(200);
    logger.xassert(ds.year() == t2.year, "Year not match");
    ds.setCentury(t2.century);
    Delay::wait(200);
    logger.xassert(ds.century() == t2.century, "Century not match");

    logger.writeLine("** AM/PM switch test case ***");
    //printTime(ds, logger);

    ds.switchTo24();
    Delay::wait(200);
    // printTime(ds, logger);
    logger.xassert(ds.hour() == t2.hour + 12, "Hour not match: %lu!=%lu", ds.hour(), t2.hour + 12);
    logger.xassert(ds.asAMPM() == false, "asAMPM->0 not match");
    logger.xassert(ds.pm() == false, "pm->0 not match");
    ds.switchToAMPM();
    Delay::wait(200);
    logger.xassert(ds.hour() == t2.hour, "Hour not match");
    logger.xassert(ds.asAMPM() == t2.asAMPM, "asAMPM->1 not match");
    logger.xassert(ds.pm() == t2.pm, "pm->1 not match");

    ds.set32kHzEnabled(true);
    Delay::wait(200);
    logger.xassert(ds.is32kHzEnabled() == true, "32kHz != true");

    ds.set32kHzEnabled(false);
    Delay::wait(200);
    logger.xassert(ds.is32kHzEnabled() == false, "32kHz != false");

    ds.setRate(2);
    Delay::wait(200);
    logger.xassert(ds.rate() == 2, "rate != 2");

    ds.setBatteryBackedSquareWaveEnabled(true);
    Delay::wait(200);
    logger.xassert(ds.batteryBackedSquareWaveEnabled() == true, "BBSW != true");

    ds.setBatteryBackedSquareWaveEnabled(false);
    Delay::wait(200);
    logger.xassert(ds.batteryBackedSquareWaveEnabled() == false, "BBSW != false");

    ds.setConvertTemperature(true);
    Delay::wait(200);
    logger.xassert(ds.convertTemperatureEnabled() == true, "convT != true");

    ds.setConvertTemperature(false);
    Delay::wait(200);
    logger.xassert(ds.convertTemperatureEnabled() == false, "convT != false");

    ds.setInterruptEnabled(true);
    Delay::wait(200);
    logger.xassert(ds.interruptEnabled() == true, "INT != true");

    ds.setInterruptEnabled(false);
    Delay::wait(200);
    logger.xassert(ds.interruptEnabled() == false, "INT != false");

    uint8_t a0 = ds.aging();
    uint8_t a1 = 25;
    ds.setAging(a1);
    Delay::wait(200);
    logger.xassert(ds.aging() == a1, "Set aging1 failed");

    ds.setAging(a0);
    Delay::wait(200);
    logger.xassert(ds.aging() == a0, "Set aging backward failed");

    /*** END OF TEST SUITE ***/
    // some demo methods
    ds.clear();
    ds.setDateTime(t);
    ds.setRate(0);
#endif // DS3231_FULL_TEST
    setClockAlarm(ds);
    // disable 32kHz to prevent false alarm
    ds.set32kHzEnabled(false);
    clock32 = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        auto t = ds.dateTime();
        auto temp = ds.temperature();
        sprintf(buf,
                "%d c, %02d.%02d.%02d %02d:%02d:%02d, pm=%d, asAMPM=%d, wd=%d; t=%d.%02d, 32kHz=%lu\r\n",
                t.century, t.day, t.month, t.year, t.hour, t.minute, t.second,
                t.pm, t.asAMPM, t.dayOfWeek, temp / 100, (temp < 0 ? -temp : temp) % 100,
                clock32);
        //logger.write(buf);
        if (clockInterrupt) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            logger.write(buf);
            logger.writeLine("Interrupt PA12/Clock");
            clockInterrupt = false;
            setClockAlarm(ds); // reset the alarm if we want to reuse it
        }
        HAL_Delay(1000);
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RTC_32KHZ_Pin RTC_INT_Pin */
  GPIO_InitStruct.Pin = RTC_32KHZ_Pin|RTC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
