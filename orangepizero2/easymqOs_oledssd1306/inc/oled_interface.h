#ifndef _OLED_INTERFACE_h
#define _OLED_INTERFACE_h

#ifdef __cplusplus
extern "C" {
#endif

#include "oled_common.h"
extern void oled_task(void (*func)(char*));
extern void OLED_ColorTurn(u8 i);
extern void OLED_DisplayTurn(u8 i);
extern void OLED_WR_Byte(u8 dat,u8 cmd);
extern void OLED_Set_Pos(u8 x, u8 y);
extern void OLED_Display_On(void);
extern void OLED_Display_Off(void);
extern void OLED_Clear(void);
extern void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 sizey);
extern u32 oled_pow(u8 m,u8 n);
extern void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 sizey);
extern void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 sizey);
extern void OLED_ShowChinese(u8 x,u8 y,u8 no,u8 sizey);
extern void OLED_DrawBMP(u8 x,u8 y,u8 sizex, u8 sizey,u8 BMP[]);
extern void OLED_Init(void);
#ifdef __cplusplus
}
#endif
#endif