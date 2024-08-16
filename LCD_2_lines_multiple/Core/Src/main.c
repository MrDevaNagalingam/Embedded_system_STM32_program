#include "stm32f4xx_hal.h"
#include "lcd.h"

int main(void) {
    HAL_Init();
    PORTS_init();
    LCD_init();

    // Display data on the first line
    LCD_set_cursor(0, 0);
    LCD_string("First Line");

    // Display data on the second line
    LCD_set_cursor(1, 0);
    LCD_string("Second Line");

    while (1) {
        // Loop indefinitely
    }
}
