#include "stm32f4xx.h"
#include "gpio.h"

int main(void) {
    GPIO_Init();
    while(1) {
        if (GPIOC->IDR & GPIO_IDR_IDR_13) /* if PC13 is high */
            GPIOA->BSRR = GPIO_BSRR_BR_5; /* turn off green LED */
        else
            GPIOA->BSRR = GPIO_BSRR_BS_5; /* turn on green LED */
    }
}
