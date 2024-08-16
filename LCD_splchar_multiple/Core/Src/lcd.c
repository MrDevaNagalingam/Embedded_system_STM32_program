#include "lcd.h"

void LCD_display_custom_chars(void) {
    LCD_command(0x80);  // Move cursor to the beginning of the first line
    LCD_data(0);  // Display alpha
    LCD_data(' '); // Space
    LCD_data(1);  // Display beta
    LCD_data(' '); // Space
    LCD_data(2);  // Display pi
    LCD_data(' '); // Space
    LCD_data(3);  // Display ohm
}

void LCD_init(void) {
    delayMs(30);
    LCD_command(0x30);
    delayMs(10);
    LCD_command(0x30);
    delayMs(1);
    LCD_command(0x30);
    LCD_command(0x38);  /* set 8-bit data, 2-line, 5x7 font */
    LCD_command(0x06);  /* move cursor right after each char */
    LCD_command(0x01);  /* clear screen, move cursor to home */
    LCD_command(0x0C);  /* turn on display, cursor blinking */
}

void PORTS_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Configure GPIO pins for LCD control (PB5, PB6, PB7) */
    GPIO_InitStruct.Pin = RS | RW | EN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configure GPIO pins for LCD data (PC0-PC7) */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                          GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Turn off EN and R/W */
    HAL_GPIO_WritePin(GPIOB, EN | RW, GPIO_PIN_RESET);
}

void LCD_command(uint8_t command) {
    HAL_GPIO_WritePin(GPIOB, RS, GPIO_PIN_RESET);  /* RS = 0 */
    HAL_GPIO_WritePin(GPIOB, RW, GPIO_PIN_RESET);  /* R/W = 0 */

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_RESET); /* Clear all pins */
    HAL_GPIO_WritePin(GPIOC, command, GPIO_PIN_SET);        /* Put command on data bus */

    HAL_GPIO_WritePin(GPIOB, EN, GPIO_PIN_SET);  /* Pulse E high */
    delayMs(1);
    HAL_GPIO_WritePin(GPIOB, EN, GPIO_PIN_RESET); /* Clear E */

    if (command < 4) {
        delayMs(2);  /* Command 1 and 2 need up to 1.64ms */
    } else {
        delayMs(1);  /* All others 40us */
    }
}

void LCD_data(char data) {
    HAL_GPIO_WritePin(GPIOB, RS, GPIO_PIN_SET);    /* RS = 1 */
    HAL_GPIO_WritePin(GPIOB, RW, GPIO_PIN_RESET);  /* R/W = 0 */

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_RESET); /* Clear all pins */
    HAL_GPIO_WritePin(GPIOC, data, GPIO_PIN_SET);           /* Put data on data bus */

    HAL_GPIO_WritePin(GPIOB, EN, GPIO_PIN_SET);  /* Pulse E high */
    delayMs(1);
    HAL_GPIO_WritePin(GPIOB, EN, GPIO_PIN_RESET); /* Clear E */
}

void LCD_create_char(uint8_t location, uint8_t charmap[]) {
    location &= 0x07;  // We only have 8 locations 0-7
    LCD_command(0x40 | (location << 3));
    for (int i = 0; i < 8; i++) {
        LCD_data(charmap[i]);
    }
}

void delayMs(int n) {
    HAL_Delay(n);
}
