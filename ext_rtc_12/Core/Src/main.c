#include "stm32f4xx_hal.h"
#include <stdio.h>

#define SLAVE_ADDR 0x68 // DS1307 I2C address

// Function prototypes
void delay(uint32_t x);
void I2C1_init(void);
void I2C1_burstRead(uint8_t saddr, uint8_t maddr, uint16_t n, uint8_t* data);
void I2C1_burstWrite(uint8_t saddr, uint8_t maddr, uint16_t n, uint8_t* data);
int BCDtoDecimal(int);
void LCD_init(void);
void LCD_command(uint8_t command);
void LCD_nibble_write(uint8_t data, uint8_t control);
void LCD_data(uint8_t data);
void LCD_string(char str[]);
void Get_Time(void);
void Error_Handler(void);
void ConvertTo12HourFormat(uint8_t hour, char* buffer, char* ampm);

// Define LCD pin masks
#define RS GPIO_PIN_5 // PA5 mask for Register Select
#define EN GPIO_PIN_7 // PA7 mask for Enable

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
    uint8_t dayofweek;
    uint8_t dayofmonth;
    uint8_t month;
    uint8_t year;
} TIME;

TIME time;

// Initialize I2C handle
I2C_HandleTypeDef hi2c1;

int main(void) {
    HAL_Init();
    I2C1_init();
    LCD_init();
    HAL_Delay(10);

    // Set initial time and date
    uint8_t timeDateToSet[7] = {0x50, 0x31, 0x16, 0x03, 0x13, 0x08, 0x24}; // 04:22 PM, 13(AUGUST), 2024
    I2C1_burstWrite(SLAVE_ADDR, 0x00, 7, timeDateToSet);

    while (1) {
        Get_Time(); // Reading RTC data

        // Prepare time string in 12-hour format
        char timeBuffer[17];
        char time12Hour[6]; // Buffer for the time in 12-hour format
        char ampm[3];
        ConvertTo12HourFormat(time.hour, time12Hour, ampm);
        snprintf(timeBuffer, sizeof(timeBuffer), "TIME: %s:%02d %s", time12Hour, time.minutes, ampm);

        // Display time on the first row
        LCD_command(0x01); // Clear display
        HAL_Delay(2); // Delay for clear command to process
        LCD_command(0x80); // Set cursor to the beginning of the first row
        LCD_string(timeBuffer);

        // Display date on the second row
        char dateBuffer[17];
        snprintf(dateBuffer, sizeof(dateBuffer), "DATE:%02d/%02d/%04d", time.dayofmonth, time.month, 2000 + time.year);
        LCD_command(0xC0); // Set cursor to the beginning of the second row
        LCD_string(dateBuffer);

        HAL_Delay(1000); // Delay to update the display every second
    }
}

void I2C1_init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE(); // Enable I2C1 clock

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
}

void I2C1_burstWrite(uint8_t saddr, uint8_t maddr, uint16_t n, uint8_t* data) {
    HAL_I2C_Mem_Write(&hi2c1, saddr << 1, maddr, I2C_MEMADD_SIZE_8BIT, data, n, HAL_MAX_DELAY);
}

void I2C1_burstRead(uint8_t saddr, uint8_t maddr, uint16_t n, uint8_t* data) {
    HAL_I2C_Mem_Read(&hi2c1, saddr << 1, maddr, I2C_MEMADD_SIZE_8BIT, data, n, HAL_MAX_DELAY);
}

void Get_Time(void) {
    uint8_t timeDateReadback[7];
    I2C1_burstRead(SLAVE_ADDR, 0x00, 7, timeDateReadback);

    time.seconds = BCDtoDecimal(timeDateReadback[0]);
    time.minutes = BCDtoDecimal(timeDateReadback[1]);
    time.hour = BCDtoDecimal(timeDateReadback[2]);
    time.dayofweek = BCDtoDecimal(timeDateReadback[3]);
    time.dayofmonth = BCDtoDecimal(timeDateReadback[4]);
    time.month = BCDtoDecimal(timeDateReadback[5]);
    time.year = BCDtoDecimal(timeDateReadback[6]);
}

void LCD_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Configure PA5, PA6, PA7 as output
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure PC4-PC7 as output
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_Delay(20); // LCD controller reset sequence
    LCD_nibble_write(0x30, GPIO_PIN_RESET);
    HAL_Delay(5);
    LCD_nibble_write(0x30, GPIO_PIN_RESET);
    HAL_Delay(1);
    LCD_nibble_write(0x30, GPIO_PIN_RESET);
    HAL_Delay(1);
    LCD_nibble_write(0x20, GPIO_PIN_RESET); // Use 4-bit data mode
    HAL_Delay(1);
    LCD_command(0x28); // Set 4-bit data, 2-line, 5x7 font
    LCD_command(0x06); // Move cursor right
    LCD_command(0x01); // Clear screen, move cursor to home
    LCD_command(0x0C); // Turn on display, cursor off
    HAL_Delay(2);
}

void LCD_nibble_write(uint8_t data, uint8_t control) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET); // Reset PC4-PC7 data bits
    HAL_GPIO_WritePin(GPIOC, (data & 0xF0), GPIO_PIN_SET); // Set data bits 4-7
    HAL_GPIO_WritePin(GPIOA, RS, control); // RS control
    HAL_GPIO_WritePin(GPIOA, EN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, EN, GPIO_PIN_RESET);
}

void LCD_command(uint8_t command) {
    LCD_nibble_write(command & 0xF0, GPIO_PIN_RESET); // Upper nibble first
    LCD_nibble_write(command << 4, GPIO_PIN_RESET); // Then lower nibble
    HAL_Delay(command < 4 ? 2 : 1); // Commands 1 and 2 need up to 1.64ms
}

void LCD_data(uint8_t data) {
    LCD_nibble_write(data & 0xF0, GPIO_PIN_SET); // Upper nibble first
    LCD_nibble_write(data << 4, GPIO_PIN_SET); // Then lower nibble
    HAL_Delay(1);
}

void LCD_string(char str[]) {
    for (int i = 0; i < 16 && str[i] != '\0'; i++) {
        LCD_data((uint8_t)str[i]);
    }
}

int BCDtoDecimal(int BCD) {
    return (((BCD >> 4) * 10) + (BCD & 0x0F));
}

void ConvertTo12HourFormat(uint8_t hour, char* buffer, char* ampm) {
    if (hour == 0) {
        snprintf(buffer, 5, "12"); // Midnight case
        snprintf(ampm, 3, "AM");
    } else if (hour < 12) {
        snprintf(buffer, 5, "%02d", hour); // AM case
        snprintf(ampm, 3, "AM");
    } else if (hour == 12) {
        snprintf(buffer, 5, "12"); // Noon case
        snprintf(ampm, 3, "PM");
    } else {
        snprintf(buffer, 5, "%02d", hour - 12); // PM case
        snprintf(ampm, 3, "PM");
    }
}

void Error_Handler(void) {
    // Implement error handling
    while (1);
}
