#include "ihastek.h"
#include "sc32f10xx.h"
#include "Function_Init.h"

#if USE_UV
void drive_uv_on_off(unsigned char en);

#if UV_FREQUENCE_DETECT
UV_TypeDef tek_uv;

volatile unsigned long AINInterruptCounterForUV;

static tek_bool UV_interruptEnableFlag,UV_DetectOKFlag;
//static unsigned long UV_ErrorCnt;

/*******************************************************
在外部检测中断口添加   tek_uv.cnt++
并声明 extern UV_TypeDef tek_uv;
这里不采用函数调用的方式，只要是考虑8位机函数放在中断比较
影响性能。
********************************************************/
void drive_uv_int_enable(void);
void drive_uv_int_disable(void);

UV_TypeDef *core_get_uv(void)
{
    return &tek_uv;
}
#if 0
static void _core_uv_int(unsigned char en)
{
    static unsigned char   en_last = 8;
    if(en != en_last)
    {
        en_last = en;
        if(en == 0)
        {
            drive_uv_int_disable();
        }
    }
}

void core_uv_err_counter(void)  //放在1MS中断中
{
    static unsigned int  W_UV_MS;
    if(UV_interruptEnableFlag)
    {
        W_UV_MS++;
        if(W_UV_MS == 1)
        {
            tek_uv.ErrorCnt = AINInterruptCounterForUV;
            AINInterruptCounterForUV = 0;
            drive_uv_int_disable();
            UV_DetectOKFlag = 1;
        }
        else if(W_UV_MS >= 2000)
        {
            W_UV_MS = 0;
            drive_uv_int_enable();
        }
    }
}
#endif

extern volatile unsigned int UV_DetectFrquencyNum;
void core_uv_handle(void)
{
    static unsigned long long uv_detect_tick;
    static unsigned char  uv_cnt, uv_err_cnt;
	static tek_bool UVonFlag,UVErrorFlag;

	core_get_tek_power()->uv= core_get_tek_power()->on=1;
    if(core_get_tek_power()->uv && core_get_tek_power()->on)
    {
		static unsigned long  uv_err_tick;
        if(tek_uv.uv_fault)
        {
            if(Get_Sys_Tick() - uv_err_tick < 3000)
            {
                drive_uv_on_off(1);
                UVonFlag=1;
            }
            else if(Get_Sys_Tick() - uv_err_tick < 6000)
            {
                drive_uv_on_off(0);
                UVonFlag=0;
            }
			else
                uv_err_tick = Get_Sys_Tick();
		}
        else
        {
        	UVonFlag=1;
            drive_uv_on_off(1);
        }

		if(UVonFlag && Get_Sys_Tick()-uv_detect_tick>=1000)
		{
			uv_detect_tick=Get_Sys_Tick();
			 // printf("The captured cycle is %d.\r\n", 64000000UL/( TIM_ICValue.SecondRising - TIM_ICValue.FirstRising));
			  //printf("The captured duty is %d.\r\n ", TIM_ICValue.FirstFailing - TIM_ICValue.FirstRising);
			tek_uv.ErrorCnt = UV_DetectFrquencyNum*100;  //传递给TE用的
			UV_DetectFrquencyNum=0;
			printf("tek_uv.ErrorCnt = %d.\r\n", tek_uv.ErrorCnt);
			if((tek_uv.ErrorCnt >= UV_NORMAL_FREQUENCY_MIN) && (tek_uv.ErrorCnt <= UV_NORMAL_FREQUENCY_MAX))//Low_UV//High_UV
            {
                uv_cnt++;
                uv_err_cnt = 0;
                if(uv_cnt >= 10)
                {
                    uv_cnt = 0;
                    tek_uv.uv_fault = 0;
                }
            }
            else
            {
                uv_cnt = 0;
                uv_err_cnt++;
                if(uv_err_cnt >= 10)
                {
                    uv_err_cnt = 0;
                    tek_uv.uv_fault = 1;
                }
            }
            tek_uv.ErrorCnt = 0;
		}			

    }
	else
	{
		uv_detect_tick=Get_Sys_Tick();
		drive_uv_on_off(0);
        UVonFlag=0;
		UV_DetectFrquencyNum=0;
	}
}

#else
void core_uv_handle(void)
{
    if(core_get_tek_power()->on && core_get_tek_power()->uv)
    {
        drive_uv_on_off(1);
    }
    else
    {
        drive_uv_on_off(0);
    }
}
#endif
#endif

