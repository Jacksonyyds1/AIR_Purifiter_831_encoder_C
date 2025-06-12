#include "ihastek.h"
#include "tm1668.h"

#if  USE_TM1668
void drive_DIO_OUTPUT(void);
void drive_STB_OUTPUT(void);
void drive_CLK_OUTPUT(void);
void drive_STB_PIN_1(void);
void drive_STB_PIN_0(void);
void drive_CLK_PIN_1(void);
void drive_CLK_PIN_0(void);
void drive_DIO_PIN_1(void);
void drive_DIO_PIN_0(void);

static tek_u8 core_get_tm1668_brightness(void) //tm1668亮度设置
{
    return Bin(10001111);
}

static void delay_tm1668(void)
{
    volatile tek_u8 B_t;
    for(B_t = 0; B_t < 20; B_t++);
}


static void core_tm1668_send_data(tek_u8 B_data, tek_bool b_TMDATA)
{
    tek_u8 B_i;
    drive_CLK_OUTPUT();
    drive_DIO_OUTPUT();                      //DIO设为输出口
    drive_STB_OUTPUT();
    if(b_TMDATA == 0)
    {
        drive_STB_PIN_1();               //发送的是指令
        delay_tm1668();
    }
    drive_STB_PIN_0();                   //片选有效
    for(B_i = 0; B_i < 8; B_i++)
    {
        drive_CLK_PIN_0();
        if(B_data & (1 << B_i))
        {
            drive_DIO_PIN_1();
        }
        else
        {
            drive_DIO_PIN_0();
        }
        delay_tm1668();
        drive_CLK_PIN_1();
    }
}


#if USE_TM1668_KEY
static tek_u8 core_tm1668_read_data(void)
{
    tek_u8 B_i, B_j = 0;
    drive_STB_PIN_0();                       //片选有效
    drive_CLK_OUTPUT();
    drive_DIO_INPUT();                       //DIO设为输入口
    drive_STB_OUTPUT();
    for(B_i = 0; B_i < 8; B_i++)
    {
        drive_CLK_PIN_0();
        delay_tm1668();
        drive_CLK_PIN_1();
        B_j >>= 1;                      //j右移一位
        if(drive_DIO_read())
        {
            B_j |= Bin(10000000);
        }
    }
    return (B_j);
}
#endif



void core_TM_main(void)
{
    tek_u8  B_DISTEMP[14] = { 0 };
    static tek_u8  B_DISTEMP_backup[14];
    tek_u8 B_j, i;
    tek_u8 brightness;
    static tek_u32  count_tick;
    tek_bool b_tm_send = 0;
    if(Get_Sys_Tick() - count_tick >= 30)
    {
        count_tick = Get_Sys_Tick();
        #if USE_TM1668_KEY
        core_tm1668_send_data(Bin(01000010), 0);
        for(B_j = 0; B_j < 5; B_j++)
        {
            B_DISTEMP[B_j] = core_tm1668_read_data();
        }
        platform_tm1668_key(B_DISTEMP);
        #endif
        for(B_j = 0; B_j < 14; B_j++)
        {
            B_DISTEMP[B_j] = 0;
        }
        platform_tm1668_display(B_DISTEMP);
        for(i = 0; i < 14; i++)
        {
            if(B_DISTEMP_backup[i] != B_DISTEMP[i])
            {
                b_tm_send = 1;
            }
            B_DISTEMP_backup[i] = B_DISTEMP[i];
        }
        if(b_tm_send)
        {
            core_tm1668_send_data(Bin(00000011),  0);
            core_tm1668_send_data(Bin(01000000),  0);
            core_tm1668_send_data(Bin(11000000),  0);
            for(B_j = 0; B_j < 14; B_j++)
            {
                core_tm1668_send_data(B_DISTEMP[B_j], 1);
            }
            brightness = core_get_tm1668_brightness();
            core_tm1668_send_data(brightness, 0);
        }
    }
}

#endif

