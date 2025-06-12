#include "lcd.h"
#include "spiLCD_flash.h"
#include "ihastek.h"
#include "at32f403a_407_board.h"

void WriteData(uint8_t dat)
{
    LCD_CS_Clr();
    spiLCD_byte_write(dat);
    LCD_CS_Set();
}
void WriteComm(uint8_t dat)
{
    LCD_CS_Clr();
    LCD_DC_Clr();//д����
    spiLCD_byte_write(dat);
    LCD_CS_Set();
    LCD_DC_Set();//д����
}

/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_DATA(uint16_t dat)
{
    LCD_CS_Clr();
    spiLCD_byte_write(dat >> 8);
    spiLCD_byte_write(dat);
    LCD_CS_Set();
}
/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_REG(uint8_t dat)
{
    LCD_CS_Clr();
    LCD_DC_Clr();//д����
    spiLCD_byte_write(dat);
    LCD_CS_Set();
    LCD_DC_Set();//д����
}
/******************************************************************************
      ����˵����������ʼ�ͽ�����ַ
      ������ݣ�x1,x2 �����е���ʼ�ͽ�����ַ
                y1,y2 �����е���ʼ�ͽ�����ַ
      ����ֵ��  ��
******************************************************************************/
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_WR_REG(0x2a);//�е�ַ����
    LCD_WR_DATA(x1 + 0x1A);
    LCD_WR_DATA(x2 + 0x1A);
    LCD_WR_REG(0x2b);//�е�ַ����
    LCD_WR_DATA(y1 + 1);
    LCD_WR_DATA(y2 + 1);
    LCD_WR_REG(0x2c);//������д
}

/******************************************************************************
 ����˵������ָ�����������ɫ
������ݣ�xsta,ysta   ��ʼ����
        xend,yend   ��ֹ����
                        color       Ҫ������ɫ
����ֵ��  ��
******************************************************************************/
void LCD_Fill_FixedColor(uint8_t xsta, uint8_t xend, uint8_t ysta, uint8_t yend, uint16_t color)
{
    if(xsta >= LCD_W || ysta >= LCD_H || xend > LCD_W || yend > LCD_H || xsta >= xend || ysta >= yend)
    {
        return;
    }
    LCD_Address_Set(xsta, ysta, xend, yend); //������ʾ��Χ
    spiLCD_color_Fill(color, ((xend - xsta + 1) * (yend - ysta + 1)));
}

/******************************************************************************
     ����˵����LCD init
    ������ݣ�
    ����ֵ��  ��
******************************************************************************/
void DisplayLCD_Init(void)
{
    gpio_bits_set(LCD_BL_PORT, LCD_BL_PIN);

#include "lcd_init.c"

    delay_ms(100);
    LCD_Fill_FixedColor(0, LCD_W - 1, 0, LCD_H - 1, 0x000000); //fill black
}

/******************************************************************************
     ����˵������������Ļ��ʾ
    ������ݣ�
    ����ֵ��  ��
******************************************************************************/
void spiLCD_AllScreen_write(uint16_t *flash_address);
void LCD_Display_FullScreen(uint16_t *flash_address)
{
    LCD_Address_Set(0, 0, LCD_W - 1, LCD_H - 1); //������ʾ��Χ

    //p=(uint8_t *)pictureAddress;
    spiLCD_AllScreen_write((uint16_t *)flash_address);
}
