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
#include "modbus_crc.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float ieee754_to_float(uint8_t byte1, uint8_t byte2, uint8_t byte3,uint8_t byte4);
float ReadModbusParameter(uint8_t regAddressHigh, uint8_t regAddressLow, uint8_t regCountHigh, uint8_t regCountLow);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t rxData[9];
uint8_t txData[8]; // Function code 0x03 for holding registers

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

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
    STATE_WAIT_CMD_DATA,
	STATE_SEND_CMD_VALUE,
	STATE_WAIT_CMD_VALUE
} FSM_State;

FSM_State currentState = STATE_IDLE;

#define RX_BUFFER_SIZE 300

uint8_t rxBuffer[RX_BUFFER_SIZE]; // Buffer for receiving data
uint8_t responseBuffer[RX_BUFFER_SIZE];
volatile uint16_t bufferIndex = 0;
volatile uint8_t rxComplete = 0;

int commandRetryCount = 0; // Counter for retry attempts
#define MAX_RETRY_COUNT 3    // Maximum number of retry attempts

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void sendCommand(const char *cmd);
void processResponse(void);
int validateResponse(const char *expectedResponse);

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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // Start UART receive interrupt
  HAL_UART_Receive_IT(&huart1, rxBuffer, 1);

  currentState = STATE_SEND_CMD_NAME; // Start FSM by sending the first command

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		if (rxComplete) {
			processResponse();
		}

		check_BE33_connection();

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


void check_BE33_connection(void) {
    switch (currentState) {
        case STATE_SEND_CMD_NAME:
            sendCommand("CMD?NAME\r\n");
            HAL_Delay(1000); // Replace with non-blocking timer in production
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
                    HAL_Delay(1000);
                } else {
                    // Handle retry failure (e.g., log error, alert user)
                    currentState = STATE_IDLE;
                }
            }
            break;

        case STATE_SEND_CMD_RESET:
        	sendCommand("CMD+LREN=1\r\n");
        	HAL_Delay(1000);
            sendCommand("CMD+RESET=0\r\n");
            HAL_Delay(1000);
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
                    HAL_Delay(1000);
                } else {
                    // Handle retry failure
                    currentState = STATE_IDLE;
                }
            }
            break;

        case STATE_SEND_CMD_TXPWR:
            sendCommand("CMD+TXPWR=-4\r\n");
            HAL_Delay(1000);
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
                    HAL_Delay(1000);
                } else {
                    // Handle retry failure
                    currentState = STATE_IDLE;
                }
            }
            break;

        case STATE_SEND_CMD_SCANPARAM:
            sendCommand("CMD+SCANPARAM=0,50,100,10000\r\n");
            HAL_Delay(1000);
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
                    HAL_Delay(1000);
                } else {
                    // Handle retry failure
                    currentState = STATE_IDLE;
                }
            }
            break;

        case STATE_SEND_CMD_SCAN:
            sendCommand("CMD+SCAN=1\r\n");
            HAL_Delay(6000);
            currentState = STATE_WAIT_CMD_SCAN;
            break;

        case STATE_WAIT_CMD_SCAN:
            if (validateResponse("EVT+ADVRPT")) {
                currentState = STATE_SEND_CMD_CON;
                commandRetryCount = 0;
            } else {
                if (commandRetryCount < MAX_RETRY_COUNT) {
                    commandRetryCount++;
                    sendCommand("CMD+SCAN=1\r\n");
                    HAL_Delay(6000);
                } else {
                    // Handle retry failure
                    currentState = STATE_IDLE;
                }
            }
            break;

        case STATE_SEND_CMD_CON:
            sendCommand("CMD+CON=1,fab321a20744\r\n");
            HAL_Delay(10000);
            currentState = STATE_WAIT_CMD_CON;
            break;

        case STATE_WAIT_CMD_CON:
            if (validateResponse("EVT+CON") && validateResponse("EVT+NOTIFY")) {
                currentState = STATE_SEND_CMD_DATA;
                commandRetryCount = 0;
            } else {
                if (commandRetryCount < MAX_RETRY_COUNT) {
                    commandRetryCount++;
                    sendCommand("CMD+CON=1,fab321a20744\r\n");
                    HAL_Delay(10000);
                } else {
                    // Handle retry failure
                    currentState = STATE_IDLE;
                }
            }
            break;

        case STATE_SEND_CMD_DATA:

            sendCommand("CMD+DATA=<conn_handle>,WELCOME TO EVERY ONE\r\n");
            HAL_Delay(1000);

            currentState = STATE_SEND_CMD_VALUE;
            break;

        case STATE_SEND_CMD_VALUE:
        	EnergyMeterValue();
        	currentState = STATE_SEND_CMD_VALUE;
        	break;

        case STATE_IDLE:
        default:
            break;
    }
}


void sendCommand(const char *cmd) {
    HAL_UART_Transmit(&huart1, (uint8_t*) cmd, strlen(cmd), HAL_MAX_DELAY);
}

void processResponse(void) {
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
            } else {
                // Handle unexpected response or retry
            }
            break;

        case STATE_WAIT_CMD_RESET:
            if (validateResponse("EVT+READY")) {
                currentState = STATE_SEND_CMD_TXPWR;
                commandRetryCount = 0;
            } else {
                // Handle unexpected response or retry
            }
            break;

        case STATE_WAIT_CMD_TXPWR:
            if (validateResponse("RSP=0")) {
                currentState = STATE_SEND_CMD_SCANPARAM;
                commandRetryCount = 0;
            } else {
                // Handle unexpected response or retry
            }
            break;

        case STATE_WAIT_CMD_SCANPARAM:
            if (validateResponse("RSP=0")) {
                currentState = STATE_SEND_CMD_SCAN;
                commandRetryCount = 0;
            } else {
                // Handle unexpected response or retry
            }
            break;

        case STATE_WAIT_CMD_SCAN:
            if (validateResponse("EVT+ADVRPT")) {
                currentState = STATE_SEND_CMD_CON;
                commandRetryCount = 0;
            } else {
                // Handle unexpected response or retry
            }
            break;

        case STATE_WAIT_CMD_CON:
            if (validateResponse("EVT+CON")) {
                currentState = STATE_SEND_CMD_DATA;
                commandRetryCount = 0;
            } else {
                // Handle unexpected response or retry
            }
            break;

        case STATE_WAIT_CMD_DATA:
            // Handle CMD+DATA responses if necessary
            break;

        default:
            break;
    }

    // Clear buffer and reset index
    bufferIndex = 0;
    memset(responseBuffer, 0, RX_BUFFER_SIZE);
    rxComplete = 0;
}

void EnergyMeterValue(void){
    char cmd[100];

    float voltage = ReadModbusParameter(0x00, 0x00, 0x00, 0x02);
    snprintf(cmd, sizeof(cmd), "CMD+DATA=<conn_handle>,Voltage: %.2f V\r\n", voltage);
    sendCommand(cmd);
    HAL_Delay(1000);

    // Read and print current
    float current = ReadModbusParameter(0x00, 0x06, 0x00, 0x02);
    snprintf(cmd, sizeof(cmd), "CMD+DATA=<conn_handle>,Current: %.2f A\r\n", current);
    sendCommand(cmd);
    HAL_Delay(1000);

    // Read and print frequency
    float frequency = ReadModbusParameter(0x00, 0x46, 0x00, 0x02);
    snprintf(cmd, sizeof(cmd), "CMD+DATA=<conn_handle>,Frequency: %.2f Hz\r\n", frequency);
    sendCommand(cmd);
    HAL_Delay(1000);

    // Read and print power
    float power = ReadModbusParameter(0x00, 0x0C, 0x00, 0x02);
     snprintf(cmd, sizeof(cmd), "CMD+DATA=<conn_handle>,power: %.2f Watt\r\n", power);
    sendCommand(cmd);
    HAL_Delay(1000);

    // Read and print power factor
    float powerFactor = ReadModbusParameter(0x00, 0x1E, 0x00, 0x02);
    snprintf(cmd, sizeof(cmd), "CMD+DATA=<conn_handle>,powerFactor: %.2f \r\n", powerFactor);
    sendCommand(cmd);
    HAL_Delay(1000);

    // Read and print Energy
    float energy = ReadModbusParameter(0x00, 0x6E, 0x00, 0x02);
    snprintf(cmd, sizeof(cmd), "CMD+DATA=<conn_handle>,Energy: %.2f KWh\r\n", energy/1000);
    sendCommand(cmd);
    HAL_Delay(1000);

    // Read and print Energy
    float rpm = ReadModbusParameter(0x00, 0x88, 0x00, 0x02);
	snprintf(cmd, sizeof(cmd), "CMD+DATA=<conn_handle>,RMP: %.2f RPM\r\n", rpm);
	sendCommand(cmd);
	HAL_Delay(1000);
}

int validateResponse(const char *expectedResponse) {
    // Check if the response buffer contains the expected response
    return strstr((char*) responseBuffer, expectedResponse) != NULL;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
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



// Function to send a Modbus request and receive the response
float ReadModbusParameter(uint8_t regAddressHigh, uint8_t regAddressLow, uint8_t regCountHigh, uint8_t regCountLow) {
    uint8_t txData[8];
    uint8_t rxData[9];

    txData[0] = 0x01; // Slave Address
    txData[1] = 0x03; // Function Code
    txData[2] = regAddressHigh;
    txData[3] = regAddressLow;
    txData[4] = regCountHigh;
    txData[5] = regCountLow;

    uint16_t crc = crc16(txData, 6);
    txData[6] = crc & 0xFF;
    txData[7] = (crc >> 8) & 0xFF;

    if (HAL_UART_Transmit(&huart3, txData, sizeof(txData), 1000) == HAL_OK) {
        //Print_Raw_Data(txData, sizeof(txData));
        if (HAL_UART_Receive(&huart3, rxData, sizeof(rxData), 1000) == HAL_OK) {
            //Print_Raw_Data(rxData, sizeof(rxData));
            uint8_t byte1 = rxData[3];
            uint8_t byte2 = rxData[4];
            uint8_t byte3 = rxData[5];
            uint8_t byte4 = rxData[6];
            return ieee754_to_float(byte1, byte2, byte3, byte4);
        } else {
            printf("Failed to receive data\n");
            return -1;
        }
    } else {
        printf("Failed to transmit data\n");
        return -1;
    }
}

// Function to convert 32-bit IEEE 754 floating-point number to float
float ieee754_to_float(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
    uint32_t raw_value = (byte1 << 24) | (byte2 << 16) | (byte3 << 8) | byte4;

    int sign = (raw_value >> 31) & 0x1;
    int exponent = (raw_value >> 23) & 0xFF;
    uint32_t mantissa = raw_value & 0x7FFFFF;

    float normalized_mantissa = 1 + (float)mantissa / (1 << 23);
    float final_value = normalized_mantissa * (1 << (exponent - 127));

    if (sign) {
        final_value = -final_value;
    }

    return final_value;
}


int _write(int file, char *ptr, int len) {
    (void) file;
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
  while (1)
  {
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
