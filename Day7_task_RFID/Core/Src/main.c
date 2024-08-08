/*SPI CONNECTION:
 *
 * MOSI----->PA7
 * MISO----->PA6
 * RST------>PA8
 * SCK------>PA5
 * SDA/CS------>PB0
 * VCC--------->3.3V
 *
 *
 ***************************
 * I2C_LCD CONNECTION:
 *
 * SDA-------->PB9
 * SCL-------->PB8
 * VCC-------->5V
 *
 ***************************
 *
 */

#include "LiquidCrystal_PCF8574.h"
#include "delay.h"
#include "stdio.h"
#include "RFID.h"
extern void SysClockConfig(void);

uint8_t rfid_id[4];
int main(void) {
	//SysClockConfig();
	systick_init_ms(16000000);
	rc522_init();
	lcd_init();
	setCursor(0, 0);
	lcd_send_string("TESTING RFID!");
	setCursor(3, 1);
	lcd_send_string("with STM32F4");
	while (1) {
		if (rc522_checkCard(rfid_id)) {

			lcd_clear();
			char data[20];
			setCursor(0, 0);
			lcd_send_string("RFID code is");
			setCursor(0, 1);
			sprintf(data, "0x%x 0x%x 0x%x 0x%x", rfid_id[0], rfid_id[1],rfid_id[2], rfid_id[3]);
			lcd_send_string(data);
			delay(1000);
		}
		delay(100);
	}

}
