/* PC0-PC7 for LCD D0-D7, respectively. */
/* PB5 for LCD R/S */
/* PB6 for LCD R/W */
/* PB7 for LCD EN */

#include<lcd.h>





int main(void)
{

	/* initialize LCD controller */
	LCD_init();
	LCD_string("WELCOME ALL ");
			delayMs(500);
	while(1){
//		LCD_string("WELCOME ALL ");
//		delayMs(500);
	}
}





















