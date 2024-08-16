/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * PA10 RX -----> BE33 TX
  * PA9 TX ------> BE33 RX
  ******************************************************************************
  * @attention
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
#include "string.h"
#include "stdio.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

#define RX_BUFFER_SIZE 300

uint8_t rxBuffer[RX_BUFFER_SIZE];  // Buffer for receiving data
volatile uint8_t responseBuffer[RX_BUFFER_SIZE];
volatile uint16_t bufferIndex = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
void sendCommand(const char *cmd);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();

  /* Start UART receive interrupt */
  HAL_UART_Receive_IT(&huart1, rxBuffer, 1);  // Receive one byte at a time

  /* Send AT commands to BLE module */
  sendCommand("CMD?NAME\r\n");
  HAL_Delay(2000);

  sendCommand("CMD+RESET=0\r\n");
  HAL_Delay(2000);

  sendCommand("CMD+TXPWR=-4\r\n");
  HAL_Delay(2000);

  sendCommand("CMD+SCANPARAM=0,50,100,10000\r\n");
  HAL_Delay(2000);

  sendCommand("CMD+SCAN=1\r\n");
  HAL_Delay(5000);

  sendCommand("CMD+CON=1,fab321a20744\r\n");
  HAL_Delay(2000);

  sendCommand("CMD+DATA=<conn_handle>,\r\n");
  HAL_Delay(2000);

  sendCommand("CMD+DATA=<conn_handle>,WELCOME to ever one\r\n");
  HAL_Delay(2000);

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  Send an AT command via UART.
  * @param  cmd: Pointer to the command string
  * @retval None
  */
void sendCommand(const char *cmd)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    memset(responseBuffer, 0, RX_BUFFER_SIZE); // Clear buffer before receiving new data
    bufferIndex = 0; // Reset buffer index
}

/**
  * @brief  UART Receive Complete Callback.
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        responseBuffer[bufferIndex++] = rxBuffer[0];  // Store received byte

        if (rxBuffer[0] == '\n' || bufferIndex >= RX_BUFFER_SIZE - 1) {  // Check for newline or buffer overflow
            responseBuffer[bufferIndex] = '\0';  // Null-terminate the string

            // Print buffer value to SWV console
            for (int i = 0; i < bufferIndex; i++) {
                ITM_SendChar(responseBuffer[i]);
            }

            bufferIndex = 0;  // Reset buffer index for next response
            memset(responseBuffer, 0, RX_BUFFER_SIZE);  // Clear buffer
        }

        // Re-enable UART receive interrupt for the next byte
        HAL_UART_Receive_IT(&huart1, rxBuffer, 1);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}
