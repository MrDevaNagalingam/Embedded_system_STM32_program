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

#define NUM_RFID_IDS 5

typedef struct {
    uint8_t id[4];
    char name[16];
} RFIDEntry;

RFIDEntry rfid_lookup[NUM_RFID_IDS] = {
    {{0x5C, 0xF2, 0xF8, 0x37}, "AKHIL"},
    {{0xC3, 0x67, 0x7C, 0x15}, "GET INSIDE"},
    {{0xB3,0x1E ,0x80 ,0x17 }, "HI DEVA"},
    {{0x33, 0x80, 0xAD, 0xD }, "hi akhil"},
    {{0xA3, 0x1F, 0xEF, 0x11}, "hi boobathi"}
};

void display_name_for_rfid(uint8_t *rfid_id) {
    char name[16] = "Unknown";
    for (int i = 0; i < NUM_RFID_IDS; i++) {
        if (memcmp(rfid_id, rfid_lookup[i].id, 4) == 0) {
            strncpy(name, rfid_lookup[i].name, sizeof(name) - 1);
            break;
        }
    }

    lcd_clear();
    setCursor(0, 0);
    lcd_send_string("WELCOME :");
    setCursor(0, 1);
    lcd_send_string(name);
}

void display_debug_info(uint8_t *rfid_id) {
    char debug_info[40];
    sprintf(debug_info, "ID: 0x%x 0x%x 0x%x 0x%x", rfid_id[0], rfid_id[1], rfid_id[2], rfid_id[3]);
    lcd_clear();
    setCursor(0, 0);
    lcd_send_string(debug_info);
}

extern void SysClockConfig(void);

int main(void) {
    // SysClockConfig();  // Uncomment if needed
    systick_init_ms(16000000);
    rc522_init();
    lcd_init();

    // Display Initial Message
    setCursor(0, 0);
    lcd_send_string("SCAN TO ACCESS!");
    setCursor(0, 1);
    lcd_send_string("TAP THE CARD");

    uint8_t rfid_id[4];

    while (1) {
        if (rc522_checkCard(rfid_id)) {
            display_name_for_rfid(rfid_id);
            delay(1000);  // Delay to avoid continuous updates
        }
        delay(100);  // Polling delay
    }
}
