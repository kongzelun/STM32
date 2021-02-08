#include "lgdp4535.h"

LCD_TypeDef LCD = {
    .Init = LGDP4535_Init,
    .DisplayOn = LGDP4535_DisplayOn,
    .DisplayOff = LGDP4535_DisplayOff,
    .SetCursor = LGDP4535_SetCursor,
    .WritePixel = LGDP4535_WritePixel,
    .Refresh = LGDP4535_Refresh,
    .ReadPixel = LGDP4535_ReadPixel,
    .SetDisplayWindow = LGDP4535_SetDisplayWindow,
};

void LGDP4535_Init(void)
{
    LCD_IO_WriteReg(0x15, 0x0030);
    LCD_IO_WriteReg(0x9A, 0x0010);
    LCD_IO_WriteReg(0x11, 0x0020);
    LCD_IO_WriteReg(0x10, 0x3428);
    LCD_IO_WriteReg(0x12, 0x0002);
    LCD_IO_WriteReg(0x13, 0x1038);
    LCD_Delay(40);

    LCD_IO_WriteReg(0x12, 0x0012);
    LCD_Delay(40);

    LCD_IO_WriteReg(0x10, 0x3420);
    LCD_IO_WriteReg(0x13, 0x3038);
    LCD_Delay(70);

    LCD_IO_WriteReg(0x30, 0x0000);
    LCD_IO_WriteReg(0x31, 0x0402);
    LCD_IO_WriteReg(0x32, 0x0307);
    LCD_IO_WriteReg(0x33, 0x0304);
    LCD_IO_WriteReg(0x34, 0x0004);
    LCD_IO_WriteReg(0x35, 0x0401);
    LCD_IO_WriteReg(0x36, 0x0707);
    LCD_IO_WriteReg(0x37, 0x0305);
    LCD_IO_WriteReg(0x38, 0x0610);
    LCD_IO_WriteReg(0x39, 0x0610);
    LCD_IO_WriteReg(0x01, 0x0100);
    LCD_IO_WriteReg(0x02, 0x0300);
    LCD_IO_WriteReg(0x03, 0x0030);
    LCD_IO_WriteReg(0x08, 0x0808);
    LCD_IO_WriteReg(0x0A, 0x0008);
    LCD_IO_WriteReg(0x60, 0x2700);
    LCD_IO_WriteReg(0x61, 0x0001);
    LCD_IO_WriteReg(0x90, 0x013E);
    LCD_IO_WriteReg(0x92, 0x0100);
    LCD_IO_WriteReg(0x93, 0x0100);
    LCD_IO_WriteReg(0xA0, 0x3000);
    LCD_IO_WriteReg(0xA3, 0x0010);

    LCD_IO_WriteReg(0x07, 0x0001);
    LCD_IO_WriteReg(0x07, 0x0021);
    LCD_IO_WriteReg(0x07, 0x0023);
    LCD_IO_WriteReg(0x07, 0x0033);
    LCD_IO_WriteReg(0x07, 0x0133);

    LCD_BackLightOn();
}

void LGDP4535_DisplayOn(void)
{
    LCD_IO_WriteReg(0x10, 0x0008);

    // power on sequence
    LCD_IO_WriteReg(0x15, 0x0030);
    LCD_IO_WriteReg(0x9A, 0x0010);
    LCD_IO_WriteReg(0x11, 0x0020);
    LCD_IO_WriteReg(0x10, 0x3428);
    LCD_IO_WriteReg(0x12, 0x0002);
    LCD_IO_WriteReg(0x13, 0x1038);
    LCD_Delay(40);

    LCD_IO_WriteReg(0x12, 0x0012);
    LCD_Delay(40);

    LCD_IO_WriteReg(0x10, 0x3420);
    LCD_IO_WriteReg(0x13, 0x3038);
    LCD_Delay(70);

    // display on sequence
    LCD_IO_WriteReg(0x07, 0x0001);
    LCD_IO_WriteReg(0x07, 0x0021);
    LCD_IO_WriteReg(0x07, 0x0023);
    LCD_IO_WriteReg(0x07, 0x0033);
    LCD_IO_WriteReg(0x07, 0x0133);
}

void LGDP4535_DisplayOff(void)
{
    LCD_IO_WriteReg(0x07, 0x0032);
    LCD_Delay(20);
    LCD_IO_WriteReg(0x07, 0x0022);
    LCD_Delay(20);
    LCD_IO_WriteReg(0x07, 0x0002);
    LCD_Delay(20);
    LCD_IO_WriteReg(0x07, 0x0000);
    LCD_Delay(10);
    LCD_IO_WriteReg(0x17, 0x0001);
    LCD_IO_WriteReg(0x13, 0x0000);
    LCD_IO_WriteReg(0x11, 0x0000);
    LCD_IO_WriteReg(0x10, 0x0008);
    LCD_Delay(10);
    LCD_IO_WriteReg(0x10, 0x000A);
}

void LGDP4535_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
    // LCD_IO_WriteReg(0x20, Xpos & 0x00FF);
    // LCD_IO_WriteReg(0x20, (Xpos >> 8) & 0x00FF);
    // LCD_IO_WriteReg(0x21, Ypos & 0x00FF);
    // LCD_IO_WriteReg(0x21, (Ypos >> 8) & 0x00FF);
    LCD_IO_WriteReg(0x20, Xpos);
    LCD_IO_WriteReg(0x21, Ypos);
}

void LGDP4535_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode)
{
    LGDP4535_SetCursor(Xpos, Ypos);
    LCD_IO_WriteReg(0x22, RGBCode);
}

void LGDP4535_Refresh(uint16_t RGBCode)
{
    LGDP4535_SetCursor(0, 0);
    LCD_IO_WriteCommand(0x22);

    for (int i = 0; i < 76800; i++)
    {
        LCD_IO_WriteData(RGBCode);
    }
}

uint16_t LGDP4535_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
    LGDP4535_SetCursor(Xpos, Ypos);
    LCD_IO_WriteCommand(0x22);
    return (LCD_IO_ReadData());
}

void LGDP4535_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
    // LCD_IO_WriteReg(0x50, Xpos & 0x00FF);
    // LCD_IO_WriteReg(0x50, (Xpos >> 8) & 0x00FF);
    // LCD_IO_WriteReg(0x51, (Xpos + Width) & 0x00FF);
    // LCD_IO_WriteReg(0x51, ((Xpos + Width) >> 8) & 0x00FF);
    // LCD_IO_WriteReg(0x52, Ypos & 0x00FF);
    // LCD_IO_WriteReg(0x52, (Ypos >> 8) & 0x00FF);
    // LCD_IO_WriteReg(0x53, (Ypos + Height) & 0x00FF);
    // LCD_IO_WriteReg(0x53, ((Ypos + Height) >> 8) & 0x00FF);
    LCD_IO_WriteReg(0x50, Xpos);
    LCD_IO_WriteReg(0x51, Xpos + Width);
    LCD_IO_WriteReg(0x52, Ypos);
    LCD_IO_WriteReg(0x53, Ypos + Height);
}
