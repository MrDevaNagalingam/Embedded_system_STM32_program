#include "stm32f4xx.h"
void delayMs(int n);
int main(void) {
RCC->AHB1ENR |= 1; /* enable GPIOA clock */
GPIOA->AFR[0] |= 0x00300000; /* PA5 pin for TIM8 */
GPIOA->MODER &=~0x00000C00;
GPIOA->MODER |= 0x00000800;
TIM8->CCR1 = TIM8->CCR1 * 110 / 100;
if (TIM8->CCR1 > 26666){/* setup TIM8 */
RCC->APB2ENR |= 2; /* enable TIM8 clock */
TIM8->PSC = 10 - 1; /*divided by 10 */
TIM8->ARR = 26667 - 1; /* divided by 26667 */
TIM8->CNT = 0;
TIM8->CCMR1 = 0x0068; /* PWM mode */
TIM8->CCER = 4; /* enable PWMCh1N */
TIM8->CCR1 = 90; /* pulse width */
TIM8->BDTR |= 0x8000; /*enable output */
TIM8->CR1 = 1; /* enable timer */
TIM8->CCR1 = 90;
delayMs(50);
}
}
/* 16 MHz SYSCLK */
void delayMs(int n) {
int i;
for (; n > 0; n--)
for (i = 0; i < 3195; i++) ;
}
