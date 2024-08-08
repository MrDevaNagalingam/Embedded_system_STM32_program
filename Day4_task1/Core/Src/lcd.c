
#include "stm32f4xx.h"

#define RS 0x20 		/* PB5 mask for reg select */
#define RW 0x40 		/* PB6 mask for read/write */
#define EN 0x80 		/* PB7 mask for enable */


void delayMs(int n);
void LCD_command(unsigned char command);
void LCD_string(char []);
void LCD_data(char data);
void LCD_init(void);
void PORTS_init(void);



void LCD_string(char str[])
{
	for(int i=0;str[i]!='\0';i++)
	{
		LCD_data(str[i]);
	}

}


/* initialize port pins then initialize LCD controller */
void LCD_init(void)
{
	PORTS_init();
	delayMs(30); /* initialization sequence */
	LCD_command(0x30);
	delayMs(10);
	LCD_command(0x30);
	delayMs(1);
	LCD_command(0x30);
	LCD_command(0x38);		/*set 8-bit data, 2-line, 5x7 font */
	LCD_command(0x06);		/*move cursor right after each char */
	LCD_command(0x01);		/*clear screen, move cursor to home */
	LCD_command(0x0F);		/*turn on display, cursor blinking */


}

void PORTS_init(void)
{
	RCC->AHB1ENR |= 0x06; 			/* enable GPIOB/C clock */
	GPIOB->MODER &= ~0x0000FC00; 		/* clear pin mode */
	GPIOB->MODER |= 0x00005400; 		/* set pin output mode */
	GPIOB->BSRR = 0x00C00000; 		/*turn off EN and R/W */


	GPIOC->MODER &= ~0x0000FFFF; 		/* clear pin mode */
	GPIOC->MODER |= 0x00005555; 		/* set pin output mode */
}

void LCD_command(unsigned char command)
{

	GPIOB->BSRR = (RS | RW) << 16; 		/* RS = 0, R/W = 0 */
	GPIOC->ODR =command; 				/* put command on data bus */
	GPIOB->BSRR = EN; 				/* pulse E high */
	delayMs(0);
	GPIOB->BSRR = EN << 16; 			/* clear E */
	if (command < 4)
		delayMs(2); 				/* command 1 and 2 needs up to 1.64ms */
	else
		delayMs(1); 				/* all others 40 us */
}

void LCD_data(char data)
{
	GPIOB->BSRR = RS; 			/* RS = 1 */
	GPIOB->BSRR = RW << 16; 		/* R/W = 0 */
	delayMs(1);

	GPIOC->ODR = data; 			/* put data on data bus */
	GPIOB->BSRR = EN; 			/*pulse E high */
	delayMs(0);
	GPIOB->BSRR = EN << 16; 		/* clear E */

}


/* delay n milliseconds (16 MHz CPU clock) */

void delayMs(int n)
{
	int i;
	for (; n > 0; n--)
		for (i = 0; i < 3195; i++) ;
}
