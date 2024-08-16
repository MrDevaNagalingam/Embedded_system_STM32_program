#ifndef LCD_H
#define LCD_H

#include "stm32f4xx_hal.h"

#define RS GPIO_PIN_5  /* PB5 mask for reg select */
#define RW GPIO_PIN_6  /* PB6 mask for read/write */
#define EN GPIO_PIN_7  /* PB7 mask for enable */

void delayMs(int n);
void LCD_command(uint8_t command);
void LCD_data(char data);
void LCD_init(void);
void PORTS_init(void);
void LCD_create_char(uint8_t location, uint8_t charmap[]);
void LCD_display_custom_chars(void);

#endif // LCD_H
