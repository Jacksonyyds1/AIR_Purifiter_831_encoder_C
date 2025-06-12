#include "ihastek.h"


#if USE_ZERO

TEKZERO_t  Zero;

void core_CrossZero_Detect(void)//����1MS��ʱ�ж���
{
    Zero.Fre_CNT++;     //cal fre
    if(drive_get_zero_pin())
    {
        Zero.Hi_num++;
        Zero.Lo_num = 0;
        if(Zero.Hi_num > 2)
        {
            Zero.Hi_num = 0;
            if(Zero.Low)
            {
                Zero.Low = 0;
                core_get_tek_power()->lost = 0;
                core_get_tek_power()->lost_cnt = 0;
                Zero.fre = 50;
                if(Zero.Fre_CNT <= 17)  { Zero.fre = 60; }
                Zero.Fre_CNT = 0;
                Zero.OneSecond_CNT++;
                if(Zero.OneSecond_CNT >= Zero.fre)  //
                {
                    Zero.OneSecond_CNT = 0;         //1S
                    Zero.seconds++;
                }
            }
        }
    }
    else
    {
        Zero.Hi_num = 0;
        Zero.Lo_num++;
        if(Zero.Lo_num > 2)
        {
            Zero.Lo_num = 0;
            Zero.Low = 1;
        }
    }
}

void core_zero_set_pin(tek_u8 pin)
{
    Zero.pin = pin;//TE检测高低电平从这里获得
}


void core_te_zero(void)
{
    Zero.pin = drive_get_zero_pin;
    core_zero_te_register(&Zero);
}


tek_u32 core_get_zero_seconds(void)
{
    return Zero.seconds;
}

void core_set_zero_seconds(tek_u32 sec)
{
    Zero.seconds = sec;
}


#endif

