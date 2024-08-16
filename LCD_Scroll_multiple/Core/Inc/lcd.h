#ifndef LCD_H
#define LCD_H

#include "stm32f4xx_hal.h"

#define RS GPIO_PIN_5  /* PB5 mask for reg select */
#define RW GPIO_PIN_6  /* PB6 mask for read/write */
#define EN GPIO_PIN_7  /* PB7 mask for enable */

void delayMs(int n);
void LCD_command(uint8_t command);
void LCD_string(char str[]);
void LCD_data(char data);
void LCD_init(void);
void PORTS_init(void);
void LCD_scroll(void);

#endif // LCD_H
