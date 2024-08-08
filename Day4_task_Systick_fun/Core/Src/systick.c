/*
 * systick.c
 *
 *  Created on: Aug 1, 2024
 *      Author: deva
 */

#include "stm32f4xx.h"

volatile uint32_t ms_ticks = 0;
volatile uint32_t us_ticks = 0;


#define SYSTICK_LOAD_VAL			16000
#define	CTRL_ENABLE					(1U<<0)
#define CTRL_CLKSRC					(1U<<2)
#define CTRL_COUNTFLAG				(1U<<16)


void systickDelayMs(int delay)
{
	/*Reload with number of clocks per millisecond*/
	SysTick->LOAD	= SYSTICK_LOAD_VAL;

	/*Clear systick current value register */
	SysTick->VAL = 0;

	/*Enable systick and select internal clk src*/
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;

	for(int i=0; i<delay ; i++){

		/*Wait until the COUNTFLAG is set*/

		while((SysTick->CTRL &  CTRL_COUNTFLAG) == 0){}

	}

	SysTick->CTRL = 0;

}

void delay_ms(uint32_t ms)
{
    uint32_t start_tick = ms_ticks;
    while ((ms_ticks - start_tick) < ms);
}

void delay_us(uint32_t us)
{
    uint32_t start_tick = us_ticks;
    uint32_t ticks_needed = us;

    while ((us_ticks - start_tick) < ticks_needed);
}
