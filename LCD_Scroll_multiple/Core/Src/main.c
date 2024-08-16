#include "stm32f4xx_hal.h"
#include "lcd.h"

int main(void) {
    HAL_Init();
    PORTS_init();
    LCD_init();

    LCD_string("WELCOME ALL");

    while (1) {
        LCD_scroll();
        delayMs(500);  // Adjust the delay for scroll speed
    }
}
