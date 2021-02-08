#ifndef __LCDDEV_H
#define __LCDDEV_H

#include "main.h"
#include "stm32f1xx_hal.h"

typedef struct
{
    void (*Init)(void);
    void (*DisplayOn)(void);
    void (*DisplayOff)(void);
    void (*SetCursor)(uint16_t Xpos, uint16_t Ypos);
    void (*WritePixel)(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode);
    void (*Refresh)(uint16_t RGBCode);
    uint16_t (*ReadPixel)(uint16_t Xpos, uint16_t Ypos);
    void (*SetDisplayWindow)(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
} LCD_TypeDef;

extern LCD_TypeDef LCD;

void LCD_IO_WriteCommand(uint16_t command);
void LCD_IO_WriteData(uint16_t data);
void LCD_IO_WriteReg(uint16_t reg, uint16_t data);
uint16_t LCD_IO_ReadData(void);
void LCD_BackLightOff(void);
void LCD_BackLightOn(void);
__weak __INLINE void LCD_Delay(__IO uint32_t Delay);

#endif
