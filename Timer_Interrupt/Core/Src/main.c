#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "uart.h"
#include "adc.h"
#include "systick.h"
#include "tim.h"



#define GPIOAEN				   (1U<<0)
#define PIN5				   (1U<<5)

#define LED				   		PIN5

static void tim2_callback(void);

int main(void)
{


	/*1.Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*2.Set PA5 as output pin*/
	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &=~(1U<<11);


	uart2_tx_init();
	tim2_1hz_interrupt_init();


	while(1)
	{


	}

}

static void tim2_callback(void)
{
	printf("A second passed !! \n\r");
	GPIOA->ODR ^=LED;
}

void TIM2_IRQHandler(void) {

	/*Clear update interrupt flag*/
	TIM2->SR &=~SR_UIF;

	//Do something..
	tim2_callback();

}




