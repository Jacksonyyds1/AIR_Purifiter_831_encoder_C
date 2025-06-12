#ifndef __LCD_H__
#define __LCD_H__

#include "at32f403a_407.h"
#include "define.h"
#include "display.h"


#define LCD_W 240
#define LCD_H 240

#define PIC_COUNTER_BYTE	2
#define PIC_DATA_BYTE			10

#define LCD_RES_Clr()  gpio_bits_reset(LCD_RES_PORT,LCD_RES_PIN)//RES
#define LCD_RES_Set()  gpio_bits_set(LCD_RES_PORT,LCD_RES_PIN)

#define LCD_CS_Clr()  gpio_bits_reset(LCD_CS_PORT,LCD_CS_PIN)//CS
#define LCD_CS_Set()  gpio_bits_set(LCD_CS_PORT,LCD_CS_PIN)

#define LCD_DC_Clr()   gpio_bits_reset(LCD_DS_PORT,LCD_DS_PIN)//DC
#define LCD_DC_Set()   gpio_bits_set(LCD_DS_PORT,LCD_DS_PIN)

typedef struct
{
	unsigned long startAddress;
	unsigned int  wide;
	unsigned int  high;
}PictureFlashDataStr;

void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_Display_FullScreen(uint16_t *flash_address);
void drive_LCD_BackLight_set(uint16_t duty);
#endif
