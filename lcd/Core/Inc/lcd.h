#ifndef LCD_H_
#define LCD_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"

typedef GPIO_TypeDef* Lcd_PortType;
typedef uint16_t Lcd_PinType;

typedef enum {
    LCD_4_BIT_MODE,
    LCD_8_BIT_MODE
} Lcd_ModeTypeDef;

typedef struct {
    Lcd_PortType data_ports[4];
    Lcd_PinType data_pins[4];
    Lcd_PortType rs_port;
    Lcd_PinType rs_pin;
    Lcd_PortType en_port;
    Lcd_PinType en_pin;
    Lcd_ModeTypeDef mode;
} Lcd_HandleTypeDef;

Lcd_HandleTypeDef Lcd_create(Lcd_PortType port[], Lcd_PinType pin[], Lcd_PortType rs_port, Lcd_PinType rs_pin, Lcd_PortType en_port, Lcd_PinType en_pin, Lcd_ModeTypeDef mode);
void Lcd_Init(Lcd_HandleTypeDef * lcd);
void Lcd_Cmd(Lcd_HandleTypeDef * lcd, uint8_t cmd);
void Lcd_Data(Lcd_HandleTypeDef * lcd, uint8_t data);
void Lcd_cursor(Lcd_HandleTypeDef * lcd, uint8_t row, uint8_t col);
void Lcd_string(Lcd_HandleTypeDef * lcd, char *str);
void Lcd_int(Lcd_HandleTypeDef * lcd, int num);

#endif /* LCD_H_ */
