#include "lcddev.h"

#define LCD_IO_BASE ((uint32_t)0x6C0007FE)

typedef struct
{
    volatile uint16_t LCD_IO_Command;
    volatile uint16_t LCD_IO_Data;
} *LCD_IOTypeDef;

static LCD_IOTypeDef lcd = ((LCD_IOTypeDef)LCD_IO_BASE);

void LCD_IO_WriteCommand(uint16_t command)
{
    lcd->LCD_IO_Command = command;
}

void LCD_IO_WriteData(uint16_t data)
{
    lcd->LCD_IO_Data = data;
}

void LCD_IO_WriteReg(uint16_t reg, uint16_t data)
{
    lcd->LCD_IO_Command = reg;
    lcd->LCD_IO_Data = data;
}

uint16_t LCD_IO_ReadData(void)
{
    return (lcd->LCD_IO_Data);
}

void LCD_BackLightOn(void)
{
    HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);
}

void LCD_BackLightOff(void)
{
    HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);
}

__weak __INLINE void LCD_Delay(__IO uint32_t Delay)
{
  HAL_Delay(Delay);
}
