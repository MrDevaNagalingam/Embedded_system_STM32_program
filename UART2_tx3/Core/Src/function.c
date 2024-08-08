/*
 * led.c
 *
 *  Created on: Feb 16, 2024
 *      Author: deva
 */
#include <prototype.h>
void LED(void){
	RCC->AHB1ENR |= 1; /* enable GPIOA clock */
	GPIOA->MODER &= ~0x00000C00; /* clear pin mode */
	GPIOA->MODER |=0x00000400; /* set pin to output mode */
while(1)
{
	GPIOA->ODR |= 0x00000020; /* turn on LED */ delayMs(500);
	GPIOA->ODR &= ~0x00000020; /* turn off LED */ delayMs(500);
}
}

/* 16 MHz SYSCLK */
void delayMs(int n)
{
	int i;
	for (; n > 0; n--)
	for (i = 0; i < 3195; i++) ;
}
