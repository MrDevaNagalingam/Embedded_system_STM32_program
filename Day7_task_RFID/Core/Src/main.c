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
#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MFRC522_CS_PIN       GPIO_PIN_12
#define MFRC522_CS_GPIO_PORT GPIOB
#define MFRC522_RST_PIN      GPIO_PIN_4
#define MFRC522_RST_GPIO_PORT GPIOA

/* MFRC522 Commands */
#define PCD_IDLE              0x00
#define PCD_TRANSCEIVE        0x0C
#define PICC_REQIDL           0x26
#define PICC_READ             0x30

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void MFRC522_Init(void);
void MFRC522_Reset(void);
uint8_t MFRC522_Check(uint8_t* id);
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* TagType);
uint8_t MFRC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen);
void MFRC522_Anticoll(uint8_t* serNum);
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_Init();
  /* USER CODE END 2 */
  char uart_buf[50];
  int uart_buf_len;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  uint8_t id[5];
	      uint8_t status = MFRC522_Check(id);

	      if (status == 0)
	      {
	        uart_buf_len = sprintf(uart_buf, "Card ID: %02X%02X%02X%02X%02X\r\n", id[0], id[1], id[2], id[3], id[4]);
	        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, uart_buf_len, HAL_MAX_DELAY);
	      }

	      HAL_Delay(1000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* MFRC522 Initialization Function */
void MFRC522_Init(void)
{
  /* Set chip select high */
  HAL_GPIO_WritePin(MFRC522_CS_GPIO_PORT, MFRC522_CS_PIN, GPIO_PIN_SET);

  /* Perform a soft reset */
  MFRC522_Reset();

  /* Set the antenna gain to the maximum value */
  uint8_t temp = 0x04 << 4;
  uint8_t reg = 0x26;
  HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi2, &temp, 1, HAL_MAX_DELAY);

  /* Turn on the antenna */
  reg = 0x14;
  HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi2, &temp, 1, HAL_MAX_DELAY);
}

void MFRC522_Reset(void)
{
  /* Send reset command */
  uint8_t reg = 0x01;
  uint8_t data = 0x0F;
  HAL_GPIO_WritePin(MFRC522_CS_GPIO_PORT, MFRC522_CS_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi2, &data, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(MFRC522_CS_GPIO_PORT, MFRC522_CS_PIN, GPIO_PIN_SET);

  /* Wait for reset to complete */
  HAL_Delay(50);
}

uint8_t MFRC522_Check(uint8_t* id)
{
  uint8_t status = MFRC522_Request(PICC_REQIDL, id);
  if (status == 0)
  {
    MFRC522_Anticoll(id);
  }
  return status;
}

uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* TagType)
{
  uint8_t status;
  uint16_t backBits;

  HAL_GPIO_WritePin(MFRC522_CS_GPIO_PORT, MFRC522_CS_PIN, GPIO_PIN_RESET);

  uint8_t buffer[2] = {reqMode, PICC_REQIDL};
  status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 2, TagType, &backBits);

  HAL_GPIO_WritePin(MFRC522_CS_GPIO_PORT, MFRC522_CS_PIN, GPIO_PIN_SET);

  return status;
}

uint8_t MFRC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen)
{
  uint8_t status = 1;
  uint8_t irqEn = 0x77;
  uint8_t waitIRq = 0x30;
  uint8_t lastBits;
  uint16_t i;

  HAL_GPIO_WritePin(MFRC522_CS_GPIO_PORT, MFRC522_CS_PIN, GPIO_PIN_RESET);

  uint8_t reg = 0x02;
  HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi2, &irqEn, 1, HAL_MAX_DELAY);

  reg = 0x04;
  HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi2, &waitIRq, 1, HAL_MAX_DELAY);

  reg = 0x01;
  HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi2, sendData, sendLen, HAL_MAX_DELAY);

  reg = 0x07;
  HAL_SPI_Receive(&hspi2, backData, *backLen, HAL_MAX_DELAY);

  reg = 0x05;
  HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi2, &lastBits, 1, HAL_MAX_DELAY);

  HAL_GPIO_WritePin(MFRC522_CS_GPIO_PORT, MFRC522_CS_PIN, GPIO_PIN_SET);

  if (status == 0)
  {
    *backLen = (i - 1) * 8 + lastBits;
    if (*backLen == 0)
    {
      *backLen = i;
    }
  }

  return status;
}

void MFRC522_Anticoll(uint8_t* serNum)
{
  uint8_t status;
  uint8_t i;
  uint8_t serNumCheck = 0;
  uint8_t unLen;

  HAL_GPIO_WritePin(MFRC522_CS_GPIO_PORT, MFRC522_CS_PIN, GPIO_PIN_RESET);

  uint8_t buffer[2] = {PICC_READ, 0x93};
  status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 2, serNum, &unLen);

  HAL_GPIO_WritePin(MFRC522_CS_GPIO_PORT, MFRC522_CS_PIN, GPIO_PIN_SET);

  for (i = 0; i < 4; i++)
  {
    serNumCheck ^= serNum[i];
  }
  if (status != 0)
  {
    serNum[4] = serNumCheck;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
