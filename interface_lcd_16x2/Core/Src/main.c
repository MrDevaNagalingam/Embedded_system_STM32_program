#include "stm32f4xx_hal.h"
#include "lcd.h"



int main(void)
{
	 HAL_Init();
    PORTS_init();
    LCD_init();

    while (1)
    {
        LCD_string("WELCOME ALL");
        delayMs(500);
    }
}


