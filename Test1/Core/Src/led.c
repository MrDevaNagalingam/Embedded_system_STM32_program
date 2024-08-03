/*
 * led.c
 *
 *  Created on: Feb 16, 2024
 *      Author: deva
 */
#include "gpio.h"
void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; /* enable GPIOC clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* enable GPIOA clock */

    GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk; /* clear pin mode */
    GPIOA->MODER |= GPIO_MODER_MODER5_0; /* set pin to output mode */

    GPIOC->MODER &= ~GPIO_MODER_MODER13_Msk; /* clear pin mode to input mode */
}
