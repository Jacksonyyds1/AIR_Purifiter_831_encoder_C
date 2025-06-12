#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"
#include "at32f403a_407_gpio.h"
#include "spiLCD_flash.h"
#include "lcd.h"
#include "define.h"
#include "ihastek.h"
#include <string.h>
#include "TimeEvent.h"

uint16_t displayDataBuffer[LCD_W*LCD_H]={0};

// 18x18字体数据 (简化的ASCII字符集)
// 每个字符18x18像素，使用1位表示一个像素
extern const uint8_t font18x18[][41];

// 当前显示位置
static uint16_t cursor_x = 0;
static uint16_t cursor_y = 0;
static uint16_t text_color = 0xFFFF; // 白色
static uint16_t bg_color = 0x0000;   // 黑色

/**
 * 设置文本颜色
 */
void Display_SetTextColor(uint16_t color)
{
    text_color = color;
}

/**
 * 设置背景颜色
 */
void Display_SetBgColor(uint16_t color)
{
    bg_color = color;
}

/**
 * 设置光标位置
 */
void Display_SetCursor(uint16_t x, uint16_t y)
{
    cursor_x = x;
    cursor_y = y;
}

/**
 * 绘制单个像素点
 */
void Display_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if(x < LCD_W && y < LCD_H)
    {
        displayDataBuffer[y * LCD_W + x] = color;
    }
}

/**
 * 绘制单个字符
 */
void Display_DrawChar(uint16_t x, uint16_t y, char ch)
{
    uint8_t char_index = 0;
    
    // 获取字符在字体数组中的索引
    if(ch >= 0x20 && ch <= 0x7E)
    {
        char_index = ch - 0x20;
    }
    
    // 绘制字符
    for(int row = 0; row < 18; row++)
    {
        for(int col = 0; col < 18; col++)
        {
            uint8_t byte_index = (row * 18 + col) / 8;
            uint8_t bit_index = (row * 18 + col) % 8;
            
            if(font18x18[char_index][byte_index] & (0x80 >> bit_index))
            {
                Display_DrawPixel(x + col, y + row, text_color);
            }
            else
            {
                Display_DrawPixel(x + col, y + row, bg_color);
            }
        }
    }
}

/**
 * 显示字符串
 */
void Display_ShowString(uint16_t x, uint16_t y, const char* str)
{
    uint16_t start_x = x;
    
    while(*str)
    {
        if(*str == '\n')
        {
            // 换行
            x = start_x;
            y += 20; // 字符高度18 + 行间距2
        }
        else if(*str == '\r')
        {
            // 回车
            x = start_x;
        }
        else
        {
            // 检查是否需要换行
            if(x + 18 > LCD_W)
            {
                x = start_x;
                y += 20;
            }
            
            // 绘制字符
            Display_DrawChar(x, y, *str);
            x += 18; // 字符宽度
        }
        str++;
        
        // 检查是否超出屏幕底部
        if(y + 18 > LCD_H)
        {
            break;
        }
    }
    
    // 更新光标位置
    cursor_x = x;
    cursor_y = y;
}

/**
 * 在当前光标位置显示字符串
 */
void Display_Printf(const char* str)
{
    Display_ShowString(cursor_x, cursor_y, str);
}

/**
 * 清空显示缓冲区
 */
void Display_Clear(void)
{
    memset(displayDataBuffer, 0, sizeof(displayDataBuffer));
    cursor_x = 0;
    cursor_y = 0;
}

/**
 * 刷新显示到LCD
 */
void Display_Refresh(void)
{
    LCD_Display_FullScreen(displayDataBuffer);
}

/*
 *UI display handler
 *
 *
 */
void UI_Show(void)
{
    // 示例使用
    Display_Clear();
    Display_SetTextColor(0xFFFF); // 白色文字
    Display_SetBgColor(0x0000);   // 黑色背景
    
    Display_ShowString(10, 10, "Hello World!\nLine 2\nLine 3");
    Display_Refresh();
}
