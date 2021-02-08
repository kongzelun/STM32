#ifndef __OLED_H
#define __OLED_H

#include "main.h"
#include "stm32f1xx_hal.h"

#define OLED_DATA_GPIO_Port GPIOC

typedef enum {
  OLED_CHAR_SIZE_1608 = 0,
  OLED_CHAR_SIZE_1206
} OLED_CharSizeTypeDef;

void OLED_Init(void);
void OLED_Refresh(uint8_t data);
void OLED_DrawPixel(uint8_t x, uint8_t y);
void OLED_PrintChar(char ch, uint8_t x, uint8_t y, OLED_CharSizeTypeDef size);
int OLED_printf(OLED_CharSizeTypeDef size, const char *format, ...);

#endif
