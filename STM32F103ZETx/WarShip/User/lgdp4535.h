#ifndef __LGDP4535_H
#define __LGDP4535_H

#include "main.h"
#include "stm32f1xx_hal.h"
#include "lcddev.h"

void LGDP4535_Init(void);
void LGDP4535_DisplayOn(void);
void LGDP4535_DisplayOff(void);
void LGDP4535_SetCursor(uint16_t Xpos, uint16_t Ypos);
void LGDP4535_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode);
void LGDP4535_Refresh(uint16_t RGBCode);
uint16_t LGDP4535_ReadPixel(uint16_t Xpos, uint16_t Ypos);

void LGDP4535_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp);
void LGDP4535_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata);

void LGDP4535_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);

#endif
