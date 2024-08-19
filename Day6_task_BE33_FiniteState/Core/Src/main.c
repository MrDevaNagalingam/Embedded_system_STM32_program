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
#include "string.h"
#include "stdio.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

typedef enum {
    STATE_IDLE,
    STATE_SEND_CMD_NAME,
    STATE_WAIT_CMD_NAME,
    STATE_SEND_CMD_RESET,
    STATE_WAIT_CMD_RESET,
    STATE_SEND_CMD_TXPWR,
    STATE_WAIT_CMD_TXPWR,
    STATE_SEND_CMD_SCANPARAM,
    STATE_WAIT_CMD_SCANPARAM,
    STATE_SEND_CMD_SCAN,
    STATE_WAIT_CMD_SCAN,
    STATE_SEND_CMD_CON,
    STATE_WAIT_CMD_CON,
    STATE_SEND_CMD_DATA,
    STATE_WAIT_CMD_DATA
} FSM_State;

FSM_State currentState = STATE_IDLE;

#define RX_BUFFER_SIZE 300

uint8_t rxBuffer[1]; // Buffer for receiving data
uint8_t responseBuffer[RX_BUFFER_SIZE];
volatile uint16_t bufferIndex = 0;
volatile uint8_t rxComplete = 0;

int commandRetryCount = 0; // Counter for retry attempts
#define MAX_RETRY_COUNT 3    // Maximum number of retry attempts

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
void sendCommand(const char *cmd);
void processResponse(void);
int validateResponse(const char *expectedResponse);

/* Private user code ---------------------------------------------------------*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_USART1_UART_Init();

    // Start UART receive interrupt
    HAL_UART_Receive_IT(&huart1, rxBuffer, 1);

    currentState = STATE_SEND_CMD_NAME; // Start FSM by sending the first command

    while (1)
    {
        if (rxComplete) {
            processResponse();
        }

        switch (currentState) {
            case STATE_SEND_CMD_NAME:
                sendCommand("CMD?NAME\r\n");
                HAL_Delay(2000); // Use timers or other non-blocking methods in real applications
                currentState = STATE_WAIT_CMD_NAME;
                break;

            case STATE_WAIT_CMD_NAME:
                if (validateResponse("RSP=0")) {
                    currentState = STATE_SEND_CMD_RESET;
                    commandRetryCount = 0;
                } else {
                    if (commandRetryCount < MAX_RETRY_COUNT) {
                        commandRetryCount++;
                        sendCommand("CMD?NAME\r\n");
                        HAL_Delay(2000);
                    } else {
                        // Handle retry failure (e.g., log error, alert user)
                        currentState = STATE_IDLE;
                    }
                }
                break;

            case STATE_SEND_CMD_RESET:
                sendCommand("CMD+RESET=0\r\n");
                HAL_Delay(2000);
                currentState = STATE_WAIT_CMD_RESET;
                break;

            case STATE_WAIT_CMD_RESET:
                if (validateResponse("EVT+READY")) {
                    currentState = STATE_SEND_CMD_TXPWR;
                    commandRetryCount = 0;
                } else {
                    if (commandRetryCount < MAX_RETRY_COUNT) {
                        commandRetryCount++;
                        sendCommand("CMD+RESET=0\r\n");
                        HAL_Delay(2000);
                    } else {
                        // Handle retry failure
                        currentState = STATE_IDLE;
                    }
                }
                break;

            case STATE_SEND_CMD_TXPWR:
                sendCommand("CMD+TXPWR=-4\r\n");
                HAL_Delay(2000);
                currentState = STATE_WAIT_CMD_TXPWR;
                break;

            case STATE_WAIT_CMD_TXPWR:
                if (validateResponse("RSP=0")) {
                    currentState = STATE_SEND_CMD_SCANPARAM;
                    commandRetryCount = 0;
                } else {
                    if (commandRetryCount < MAX_RETRY_COUNT) {
                        commandRetryCount++;
                        sendCommand("CMD+TXPWR=-4\r\n");
                        HAL_Delay(2000);
                    } else {
                        // Handle retry failure
                        currentState = STATE_IDLE;
                    }
                }
                break;

            case STATE_SEND_CMD_SCANPARAM:
                sendCommand("CMD+SCANPARAM=0,50,100,10000\r\n");
                HAL_Delay(2000);
                currentState = STATE_WAIT_CMD_SCANPARAM;
                break;

            case STATE_WAIT_CMD_SCANPARAM:
                if (validateResponse("RSP=0")) {
                    currentState = STATE_SEND_CMD_SCAN;
                    commandRetryCount = 0;
                } else {
                    if (commandRetryCount < MAX_RETRY_COUNT) {
                        commandRetryCount++;
                        sendCommand("CMD+SCANPARAM=0,50,100,10000\r\n");
                        HAL_Delay(2000);
                    } else {
                        // Handle retry failure
                        currentState = STATE_IDLE;
                    }
                }
                break;

            case STATE_SEND_CMD_SCAN:
                sendCommand("CMD+SCAN=1\r\n");
                HAL_Delay(5000);
                currentState = STATE_WAIT_CMD_SCAN;
                break;

            case STATE_WAIT_CMD_SCAN:
                if (validateResponse("EVT+ADVRPT")) {
                    // Replace with actual expected scan response
                    currentState = STATE_SEND_CMD_CON;
                    commandRetryCount = 0;
                } else {
                    if (commandRetryCount < MAX_RETRY_COUNT) {
                        commandRetryCount++;
                        sendCommand("CMD+SCAN=1\r\n");
                        HAL_Delay(5000);
                    } else {
                        // Handle retry failure
                        currentState = STATE_IDLE;
                    }
                }
                break;

            case STATE_SEND_CMD_CON:
                sendCommand("CMD+CON=1,fab321a20744\r\n");
                HAL_Delay(2000);
                currentState = STATE_WAIT_CMD_CON;
                break;

            case STATE_WAIT_CMD_CON:
                if (validateResponse("RSP=0")) {
                    currentState = STATE_SEND_CMD_DATA;
                    commandRetryCount = 0;
                } else {
                    if (commandRetryCount < MAX_RETRY_COUNT) {
                        commandRetryCount++;
                        sendCommand("CMD+CON=1,fab321a20744\r\n");
                        HAL_Delay(2000);
                    } else {
                        // Handle retry failure
                        currentState = STATE_IDLE;
                    }
                }
                break;

            case STATE_SEND_CMD_DATA:
                sendCommand("CMD+DATA=<conn_handle>,\r\n");
                HAL_Delay(2000);
                sendCommand("CMD+DATA=<conn_handle>,WELCOME TO EVERY ONE\r\n");
                currentState = STATE_IDLE;
                break;

            case STATE_IDLE:
            default:
                break;
        }
    }
}

void sendCommand(const char *cmd)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
}

void processResponse(void)
{
    // Print the entire response buffer to debug
    for (uint16_t i = 0; i < bufferIndex; i++) {
        ITM_SendChar(responseBuffer[i]);
    }
    ITM_SendChar('\n'); // Newline for clarity

    // Check and handle different responses based on state
    switch (currentState) {
        case STATE_WAIT_CMD_NAME:
            if (validateResponse("RSP=0")) {
                currentState = STATE_SEND_CMD_RESET;
                commandRetryCount = 0;
            } else if (validateResponse("EVT+READY")) {
                // Handle unexpected response if needed
            } else {
                // Retry or handle error
            }
            break;

        case STATE_WAIT_CMD_RESET:
            if (validateResponse("EVT+READY")) {
                currentState = STATE_SEND_CMD_TXPWR;
                commandRetryCount = 0;
            } else {
                // Retry or handle error
            }
            break;

        case STATE_WAIT_CMD_TXPWR:
            if (validateResponse("RSP=0")) {
                currentState = STATE_SEND_CMD_SCANPARAM;
                commandRetryCount = 0;
            } else {
                // Retry or handle error
            }
            break;

        case STATE_WAIT_CMD_SCANPARAM:
            if (validateResponse("RSP=0")) {
                currentState = STATE_SEND_CMD_SCAN;
                commandRetryCount = 0;
            } else {
                // Retry or handle error
            }
            break;

        case STATE_WAIT_CMD_SCAN:
            if (validateResponse("EVT+ADVRPT")) {
                // Replace with actual expected scan response
                currentState = STATE_SEND_CMD_CON;
                commandRetryCount = 0;
            } else {
                // Retry or handle error
            }
            break;

        case STATE_WAIT_CMD_CON:
            if (validateResponse("EVT+CON")) {
                currentState = STATE_SEND_CMD_DATA;
                commandRetryCount = 0;
            } else {
                // Retry or handle error
            }
            break;

        case STATE_WAIT_CMD_DATA:
            // Add handling for CMD+DATA responses if necessary
            break;

        default:
            break;
    }

    // Clear buffer and reset index
    bufferIndex = 0;
    memset(responseBuffer, 0, RX_BUFFER_SIZE);
    rxComplete = 0;
}

int validateResponse(const char *expectedResponse)
{
    // Check if the response buffer contains the expected response
    return strstr((char*)responseBuffer, expectedResponse) != NULL;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        if (bufferIndex < RX_BUFFER_SIZE - 1) {
            responseBuffer[bufferIndex++] = rxBuffer[0];
        }

        if (rxBuffer[0] == '\n' || bufferIndex >= RX_BUFFER_SIZE - 1) {
            responseBuffer[bufferIndex] = '\0';
            rxComplete = 1;
        }

        HAL_UART_Receive_IT(&huart1, rxBuffer, 1);
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
