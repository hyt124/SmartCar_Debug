#ifndef __oled__
#define __oled__
#include "common.h"

    void LCD_WrDat(byte data);
    void LCD_WrCmd(byte cmd);
    void LCD_Set_Pos(byte x, byte y);    
    void LCD_Fill(byte bmp_data);
    void LCD_CLS(void);
    void LCD_DLY_ms(word ms);
    void LCD_Init(void);        
    void LCD_PutPixel(byte x,byte y);
    void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);
    void LCD_P6x8Str(char x,char y,char ch[]);
    void fsLCD_P6x8Str(char x,char y,char ch[]);
    void LCD_P6x8Char(byte x,byte y,byte ch);
    void LCD_P8x16Str(byte x,byte y,byte *ch);
    void LCD_Print(byte x, byte y, byte ch[]);
    void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]);
    void LCD_PrintValueI(byte x, byte y, int data);
    void LCD_PrintValueFP(byte x, byte y, dword data, byte num);
    void LCD_PrintValueF(byte x, byte y, float data, byte num);
    void OLed_DisplayI(byte x,byte y,int num);
    void OLed_DisplayF(byte x,byte y,float num);
#endif
