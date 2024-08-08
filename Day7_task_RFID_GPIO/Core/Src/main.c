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

/*SPI CONNECTION:
 *
 * MOSI----->PA7
 * MISO----->PA6
 * RST------>PA8
 * SCK------>PA5
 * SDA/CS------>PB0
 * VCC--------->3.3V
 *
 *
 ***************************
 * I2C_LCD CONNECTION:
 *
 * SDA-------->PB9
 * SCL-------->PB8
 * VCC-------->5V
 *
 ***************************
 *
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal_PCF8574.h"
#include "delay.h"
#include "stdio.h"
#include "RFID.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Define the number of RFID IDs
#define NUM_RFID_IDS 5

// Define the LED GPIO pins
#define LED1_PIN GPIO_PIN_0  // PC0
#define LED2_PIN GPIO_PIN_1  // PC1
#define LED3_PIN GPIO_PIN_2  // PC3

// Define the LED GPIO port
#define LED_GPIO_PORT GPIOC

// Define the RFID entry structure
typedef struct {
    uint8_t id[4];
    char name[16];
} RFIDEntry;


// Define the RFID entries
RFIDEntry rfid_lookup[NUM_RFID_IDS] = {
    {{0x5C, 0xF2, 0xF8, 0x37}, "AKHIL"},
    {{0xC3, 0x67, 0x7C, 0x15}, "GET INSIDE"},
    {{0xB3, 0x1E, 0x80, 0x17}, "HI DEVA"},
    {{0x33, 0x80, 0xAD, 0xD}, "hi akhil"},
    {{0xA3, 0x1F, 0xEF, 0x11}, "hi boobathi"}
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void turn_on_led(uint8_t *rfid_id);
void clear_leds(void);
void display_name_for_rfid(uint8_t *rfid_id);
void system_init(void);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  system_init();

     // Display Initial Message
     setCursor(0, 0);
     lcd_send_string("SCAN TO ACCESS!");
     setCursor(0, 1);
     lcd_send_string("TAP THE CARD");

     uint8_t rfid_id[4];
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if (rc522_checkCard(rfid_id)) {
	             clear_leds();  // Clear all LEDs before turning on the new one
	             turn_on_led(rfid_id);
	             display_name_for_rfid(rfid_id);
	             delay(1000);  // Delay to avoid continuous updates
	         }
	         delay(100);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

// Function to initialize the LEDs
void init_leds(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable the GPIOC clock
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Configure PC0, PC1, and PC3 as output
    GPIO_InitStruct.Pin = LED1_PIN | LED2_PIN | LED3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

    // Initially turn off all LEDs
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED1_PIN | LED2_PIN | LED3_PIN, GPIO_PIN_RESET);
}

// Function to turn on the LED based on RFID ID
void turn_on_led(uint8_t *rfid_id) {
    if (memcmp(rfid_id, rfid_lookup[0].id, 4) == 0) {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);  // Turn on LED1 (PC0)
        printf("LED1 turned on\n");
    } else if (memcmp(rfid_id, rfid_lookup[1].id, 4) == 0) {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);  // Turn on LED2 (PC1)
        printf("LED2 turned on\n");
    } else if (memcmp(rfid_id, rfid_lookup[2].id, 4) == 0) {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED3_PIN, GPIO_PIN_SET);  // Turn on LED3 (PC3)
        printf("LED3 turned on\n");
    } else {
        printf("Unknown RFID\n");
    }
}

// Function to clear all LEDs
void clear_leds(void) {
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED1_PIN | LED2_PIN | LED3_PIN, GPIO_PIN_RESET);
}

// Function to display RFID information on the LCD
void display_name_for_rfid(uint8_t *rfid_id) {
    char name[16] = "Unknown";
    for (int i = 0; i < NUM_RFID_IDS; i++) {
        if (memcmp(rfid_id, rfid_lookup[i].id, 4) == 0) {
            strncpy(name, rfid_lookup[i].name, sizeof(name) - 1);
            break;
        }
    }

    lcd_clear();
    setCursor(0, 0);
    lcd_send_string("WELCOME :");
    setCursor(0, 1);
    lcd_send_string(name);
}

// Function to initialize the system
void system_init(void) {
    systick_init_ms(16000000);
    rc522_init();
    lcd_init();
    init_leds();
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
