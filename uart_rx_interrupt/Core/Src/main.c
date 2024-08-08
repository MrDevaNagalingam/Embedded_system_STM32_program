#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "uart.h"


#define GPIOAEN				   (1U<<0)
#define GPIOA_5				   (1U<<5)

#define LED_PIN				   GPIOA_5

char key;


static void uart_callback(void);

int main(void)
{

	/*1.Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*2.Set PA5 as output pin*/
	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &= ~(1U<<11);


	uart2_rx_interrupt_init();

	while(1)
	{

	}

}

static void uart_callback(void)
{
	key =  USART2->DR;

	if(key == '1')
		{
			GPIOA->ODR |= LED_PIN;
		}
		else{
			GPIOA->ODR &= ~LED_PIN;

		}
}

void USART2_IRQHandler(void)
{
	/*ChecK if RXNE is set*/

	if(USART2->SR & SR_RXNE)
	{
		//Do something
		uart_callback();

	}
}



