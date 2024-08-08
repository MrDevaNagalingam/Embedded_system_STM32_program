#include "LiquidCrystal_PCF8574.h"
//#include "delay.h"
#include "stdio.h"
//#include "RFID.h"
extern void SysClockConfig(void);
#include"delay.h"
uint8_t rfid_id[4];
int main(void)
		{
		//SysClockConfig();
		systick_init_ms(16000000);
		//rc522_init();
		lcd_init();
		setCursor(0,0);
		lcd_send_string("DEVA");
		setCursor(4,1);
		lcd_send_string("STM32F446RE");
		delay(2000);
		lcd_clear();
		while(1)
				{
				//if(rc522_checkCard(rfid_id))
							{

							lcd_clear();
							char data[20];
							setCursor(0,0);
							lcd_send_string("DEVA N");
							setCursor(0,1);
							//sprintf(data,"0x%x 0x%x 0x%x 0x%x",rfid_id[0],rfid_id[1],rfid_id[2],rfid_id[3]);
							lcd_send_string("RTC2023MCA030");
							delay(1000);
							}
				//delay(100);
				}

		}
