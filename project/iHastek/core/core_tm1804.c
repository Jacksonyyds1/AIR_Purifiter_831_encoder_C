
#include "ihastek.h"

#if USE_TM1804


#if USE_APT32F1023H8S6
    volatile unsigned char delay_flag;
#endif


void    drive_tm1804_DIN_OUTPUT(void);

//void drive_reset_DIN_PIN(void);
//void drive_set_DIN_PIN(void);

void drive_tm1804_disable_int(void);
void drive_tm1804_enable_int(void);

static void _core_TM1804delay(void)
{
    volatile tek_u8 i;
    for(i = 0; i < 200; i++);
}

static void _core_TM1804_Reset(void)
{
    drive_reset_DIN_PIN();
    _core_TM1804delay();
}


void core_Send_tm1804_RGBValue(tek_u8 Red, tek_u8 Green, tek_u8 Blue, tek_u8 Num)
{
    tek_u8 i;
    tek_u32 colour;
    tek_u8 byte;
    colour = ((tek_u32)Red << 16) | ((tek_u32)Green << 8) | ((tek_u32)Blue);
    drive_tm1804_disable_int();
    drive_tm1804_DIN_OUTPUT();
    //---------------------RED---------------------------
    for(i = 0; i < Num; i++)
    {
        #if 0
        for(i = 24; i > 0; i--)
        {
            byte = ((colour >> (i - 1)) & 0x01);
            if(byte)
            {
                drive_set_DIN_PIN();
                TM1804_NOP_1_HIGH;
                drive_reset_DIN_PIN();
                TM1804_NOP_1_LOW;
            }
            else
            {
                drive_set_DIN_PIN();
                TM1804_NOP_0_HIGH;
                drive_reset_DIN_PIN();
                TM1804_NOP_0_LOW;
            }
        }
        #endif
        #if 1
        if(Red & 0x80)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Red & 0x40)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Red & 0x20)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Red & 0x10)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Red & 0x08)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Red & 0x04)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Red & 0x02)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Red & 0x01)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        //---------------------Green---------------------------
        if(Green & 0x80)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Green & 0x40)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Green & 0x20)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Green & 0x10)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Green & 0x08)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Green & 0x04)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Green & 0x02)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Green & 0x01)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        //---------------------BLUE---------------------------
        if(Blue & 0x80)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Blue & 0x40)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Blue & 0x20)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Blue & 0x10)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Blue & 0x08)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Blue & 0x04)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Blue & 0x02)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        if(Blue & 0x01)
        {
            drive_set_DIN_PIN();
            TM1804_NOP_1_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_1_LOW;
        }
        else
        {
            drive_set_DIN_PIN();
            TM1804_NOP_0_HIGH;
            drive_reset_DIN_PIN();
            TM1804_NOP_0_LOW;
        }
        #endif
    }
    _core_TM1804_Reset();       //·¢ËÍ¸´Î»ÐÅºÅ£¬Íê³ÉÒ»´ÎRGB´«ËÍÏÔÊ¾
    drive_tm1804_enable_int();
}
/***************************************************************************************/
#endif
