#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "at32f403a_407.h"


typedef struct{
	uint8_t x_axis_value;
	uint8_t y_axis_value;
}STR_COORDINATE;

// 新增的函数声明
void Display_SetTextColor(uint16_t color);
void Display_SetBgColor(uint16_t color);
void Display_SetCursor(uint16_t x, uint16_t y);
void Display_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void Display_DrawChar(uint16_t x, uint16_t y, char ch);
void Display_ShowString(uint16_t x, uint16_t y, const char* str);
void Display_Printf(const char* str);
void Display_Clear(void);
void Display_Refresh(void);
void UI_Show(void);

#ifdef __cplusplus
}
#endif
#endif
