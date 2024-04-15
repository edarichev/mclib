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
#define PLATFORM_STM32_HAL
#include <delay.h>
#include <xuart.h>
#include <i2cclient.h>
#include <i2cscanner.h>
#include <display/SSD1306.h>
#include <display/fonts/fontCP866_8x8.h>
#include <display/fonts/font_mono_10x18.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
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
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */

    static UARTLogger logger(&huart2);
    uint8_t ic = I2CScanner::enumerate(&hi2c1, [](auto addr) {
        logger.writeLine("I2C device found at address %#2X", addr);
    });
    logger.writeLine("%d I2C devices found", ic);
    I2CPollingModeMaster i2c(&hi2c1);
    SSD1306 display(&i2c);
    display.init();
    Delay::wait(1000);

    display.fill(0);
    for (size_t i = 0; i < 8; ++i) {
        display.setPos(i, i);
        display.printString("0", FontCP866_8x8);
    }
    Delay::wait(1000);
    display.setPos(2, 5);
    display.printString("Hello, world!!!", FontCP866_8x8);
    Delay::wait(1000);
    uint8_t lb[8] = {0x41,0x7F,0x7F,0x08,0x1C,0x77,0x63,0x00,};
    display.fillWith(lb);
    Delay::wait(1000);
    display.setGraphicsMode();
    display.fill(0);
    /*display.drawLine(0xFFFFFF, 10, 0, 10, 63);
    display.drawLine(0xFFFFFF, 0, 50, 100, 50);
    display.drawLine(0xFFFFFF, 0, 0, 127, 63);
    display.drawLine(0xFFFFFF, 20, 0, 21, 63); // "\"
    display.drawLine(0xFFFFFF, 20, 63, 21, 0); // /
    display.drawLine(0xFFFFFF, 20, 63, 19, 0); // "\"
    display.drawLine(0xFFFFFF, 20, 63, 15, 0); // "\"
    display.drawLine(0xFFFFFF, 20, 63, 10, 0); // "\"
    display.drawEllipse(0xFFFFFF, 10, 10, 100, 40);
    display.drawRectangle(0xFFFFFF, 10, 10, 100, 40);

    display.fillRectangle(0xFFFFFFFF, 50, 20, 40, 15);
    display.fillEllipse(0xFFFFFFFF, 50, 40, 40, 15);*/

    uint8_t bmp19x19[] = {
            0b10000000,
            0b01000000,
            0b00101000,
            0b00010100,
            0b00001000,
            0b10000100,
            0b01000010,
            0b00001001,
            0b00000100,
            0b10000000,
            0b11000000,
            0b01100000,
            0b00011000,
            0b00001100,
            0b00000100,
            0b10000010,
            0b01000001,
            0b00001000,
            0b10000100,
            0b01000000,
            0b10100000,
            0b01010000,
            0b00001000,
            0b00000101,
            0b00000010,
            0b10000001,
            0b00010000,
            0b10001000,
            0b01000001,
            0b00100000,
            0b10010000,
            0b00011000,
            0b00001100,
            0b00000011,
            0b00000001,
            0b10000000,
            0b10010000,
            0b01001000,
            0b00100001,
            0b00010000,
            0b10001000,
            0b00010100,
            0b00001010,
            0b00000001,
            0b00000000,
            0b10000000,
    };

    /*display.drawBitmap(7, 29, bmp19x19, 19, 19, RasterOpCode::SRCAND);
    display.drawBitmap(7 + 19, 29, bmp19x19, 19, 19, RasterOpCode::SRCCOPY);
    display.drawBitmap(7 + 19*2, 29, bmp19x19, 19, 19, RasterOpCode::SRCOR);
    display.drawBitmap(7 + 19*3, 29, bmp19x19, 19, 19, RasterOpCode::SRCXOR);*/

    char str[2] = {0, 0};
    int charWidth = 10;
    int charHeight = 18;

    uint8_t ch = 0;
    for (int a = 0; a < 8; ++a) {
        display.fill(0);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 13; c++) {
                str[0] = ch == 0 ? ' ' : ch;
                ch++;
                display.drawString(0xFFFFFFFF, c * charWidth, r * charHeight, str, Font10x18, 10, 18);
            }
        }
        display.flush();
        Delay::wait(3000);
    }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        //display.fillWith(0b11111111);
        //display.fillWith(letterA);
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
