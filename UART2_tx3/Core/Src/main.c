#include"prototype.h"
#include "stm32f4xx.h"
#include <stdint.h>

#define    GPIOAEN     (1U<<0)
#define    PA2MOD      (2U<<4)
#define    PA2AF       (7U<<8)
#define    APB1EN      (1U<<17)
#define    USART2_TX    (1U<<3)
#define    USART2EN     (1U<<13)
#define    SR_TXE       (1U<<7)


#define SYS_FREQ		16000000
#define APB1_CLK		SYS_FREQ

#define UART_BAUDRATE	115200


void uart2_init(void);
void uart2_write(const char *ch);

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk,  uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);

int main()
{

	uart2_init();
	while(1)
	{
		uart2_write("Deva \r \n");
		delayMs(100);
	}
	return 0;
}


void uart2_init(void)
{

	/****************Configure uart gpio pin***************/
		/*Enable clock access to gpioa */
		RCC->AHB1ENR |= GPIOAEN;

		/*Set PA2 mode to alternate function mode*/
		GPIOA->MODER  |= PA2MOD;

		/*Set PA2 alternate function type to UART_TX (AF07)*/
        GPIOA->AFR[0] |= PA2AF;

		/****************Configure uart module ***************/
		/*Enable clock access to uart2 */
         RCC->APB1ENR |= APB1EN;

		/*Configure baudrate*/
		uart_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE);
		//USART2->BRR = ((APB1_CLK + (BaudRate/2U))/BaudRate) ;

		/*Configure the transfer direction*/
		USART2->CR1 = USART2_TX;

		/*Enable uart module*/
        USART2->CR1 |= USART2EN;

}






static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk,  uint32_t BaudRate) {
    uint32_t usartdiv = 0;

    // Calculate USARTDIV value
    if ((USARTx == USART1) || (USARTx == USART6)) {
        usartdiv = ((2 * PeriphClk) + (BaudRate / 2)) / BaudRate;
    } else {
        // For other USARTs (USART2, USART3, UART4, UART5)
        usartdiv = ((25 * PeriphClk) + (BaudRate / 2)) / (BaudRate * 16);
    }

    // Assign BRR
    USARTx->BRR = usartdiv;
}


static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
	return ((PeriphClk + (BaudRate/2U))/BaudRate);
}






void uart2_write(const char *ch)
{
	while(*ch !='\0'){

  /*Write to transmit data register*/

	USART2->DR	=  (*ch & 0xFF);
	ch++;
	/*Make sure the transmit data register is empty*/
	while(!(USART2->SR & SR_TXE)){}

	}



}





