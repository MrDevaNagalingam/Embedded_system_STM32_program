#include "lcd.h"

void Lcd_Port(Lcd_HandleTypeDef * lcd, uint8_t data)
{
    HAL_GPIO_WritePin(lcd->data_ports[0], lcd->data_pins[0], data & 0x01);
    HAL_GPIO_WritePin(lcd->data_ports[1], lcd->data_pins[1], (data >> 1) & 0x01);
    HAL_GPIO_WritePin(lcd->data_ports[2], lcd->data_pins[2], (data >> 2) & 0x01);
    HAL_GPIO_WritePin(lcd->data_ports[3], lcd->data_pins[3], (data >> 3) & 0x01);
}

void Lcd_EnablePulse(Lcd_HandleTypeDef * lcd)
{
    HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void Lcd_Cmd(Lcd_HandleTypeDef * lcd, uint8_t cmd)
{
    HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, GPIO_PIN_RESET);
    Lcd_Port(lcd, (cmd >> 4));
    Lcd_EnablePulse(lcd);
    Lcd_Port(lcd, cmd);
    Lcd_EnablePulse(lcd);
}

void Lcd_Data(Lcd_HandleTypeDef * lcd, uint8_t data)
{
    HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, GPIO_PIN_SET);
    Lcd_Port(lcd, (data >> 4));
    Lcd_EnablePulse(lcd);
    Lcd_Port(lcd, data);
    Lcd_EnablePulse(lcd);
}

void Lcd_Init(Lcd_HandleTypeDef * lcd)
{
    HAL_Delay(50);
    Lcd_Cmd(lcd, 0x02);
    Lcd_Cmd(lcd, 0x28);
    Lcd_Cmd(lcd, 0x0C);
    Lcd_Cmd(lcd, 0x06);
    Lcd_Cmd(lcd, 0x01);
    HAL_Delay(5);
}

Lcd_HandleTypeDef Lcd_create(Lcd_PortType port[], Lcd_PinType pin[], Lcd_PortType rs_port, Lcd_PinType rs_pin, Lcd_PortType en_port, Lcd_PinType en_pin, Lcd_ModeTypeDef mode)
{
    Lcd_HandleTypeDef lcd;

    lcd.data_ports[0] = port[0];
    lcd.data_ports[1] = port[1];
    lcd.data_ports[2] = port[2];
    lcd.data_ports[3] = port[3];

    lcd.data_pins[0] = pin[0];
    lcd.data_pins[1] = pin[1];
    lcd.data_pins[2] = pin[2];
    lcd.data_pins[3] = pin[3];

    lcd.rs_port = rs_port;
    lcd.rs_pin = rs_pin;
    lcd.en_port = en_port;
    lcd.en_pin = en_pin;

    lcd.mode = mode;

    Lcd_Init(&lcd);

    return lcd;
}

void Lcd_cursor(Lcd_HandleTypeDef * lcd, uint8_t row, uint8_t col)
{
    uint8_t maskData;

    maskData = (col-1) & 0x0F;
    if(row==0)
    {
        maskData |= 0x80;
        Lcd_Cmd(lcd, maskData);
    }
    else
    {
        maskData |= 0xC0;
        Lcd_Cmd(lcd, maskData);
    }
}

void Lcd_string(Lcd_HandleTypeDef * lcd, char *str)
{
    while (*str) Lcd_Data(lcd, *str++);
}

void Lcd_int(Lcd_HandleTypeDef * lcd, int num)
{
    char str[16];
    sprintf(str, "%d", num);
    Lcd_string(lcd, str);
}
