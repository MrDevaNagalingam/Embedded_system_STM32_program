#include "stm32f4xx_hal.h"
#include "lcd.h"

int main(void) {
    HAL_Init();
    PORTS_init();
    LCD_init();

    // Define custom characters
    uint8_t alpha[8] = {0x04, 0x0A, 0x0A, 0x0E, 0x1F, 0x11, 0x11, 0x00};
    uint8_t beta[8] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E, 0x00};
    uint8_t pi[8] = {0x1F, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x00};
    uint8_t ohm[8] = {0x0E, 0x11, 0x11, 0x0E, 0x04, 0x1F, 0x04, 0x00};

    // Load custom characters into CGRAM
    LCD_create_char(0, alpha);
    LCD_create_char(1, beta);
    LCD_create_char(2, pi);
    LCD_create_char(3, ohm);

    // Display custom characters
    LCD_display_custom_chars();

    while (1) {
        // Loop indefinitely
    }
}
