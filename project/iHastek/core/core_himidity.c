#include "ihastek.h"

#if USE_HUMIDIFIER
static TEKHumiditifier_t  humidity;
static tek_u32  power_off_delay_tick;


TEKHumiditifier_t *get_humi_status(void)
{
    return &humidity;
}



#if USE_WATER_LACK_SINGLE_PIN

tek_u8 drive_get_cs_ks_pin(void);

static void _water_detect(void)
{
    static tek_u32  water_tick;
    static tek_u8  ks_hi_cnt, ks_low_cnt;
    if(Get_Sys_Tick() - water_tick >= 70)
    {
        water_tick = Get_Sys_Tick();
        if(drive_get_cs_ks_pin())
        {
            ks_hi_cnt++;
            ks_low_cnt = 0;
            if(ks_hi_cnt >= 20)
            {
                ks_hi_cnt = 0;
                humidity.low_water = 0;
                power_off_delay_tick = Get_Sys_Tick();
                humidity.power_off_delay = 0;
            }
        }
        else
        {
            ks_hi_cnt = 0;
            ks_low_cnt++;
            if(ks_low_cnt >= 20)
            {
                ks_low_cnt = 0;
                humidity.low_water = 1;
                if(core_get_tek_power()->on)
                {
                    #if USE_LOW_WATER_DELAY_OFF
                    if(core_get_tek_power()->on_secs > 180) //开机3分钟后
                    {
                        humidity.power_off_delay = 1; //启动延时关机
                    }
                    else
                    {
                        core_get_tek_power()->on = 0;
                        humidity.b_wink = 1; //启动闪烁
                        humidity.wink_tick = Get_Sys_Tick();
                        humidity.wink_time = 0;
                    }
                    #else
                    core_get_tek_power()->on = 0;
                    humidity.b_wink = 1; //启动闪烁
                    humidity.wink_tick = Get_Sys_Tick();
                    humidity.wink_time = 0;
                    #endif
                }
            }
        }
        if(humidity.burn_pluse > 20 && core_get_tek_power()->on)
        {
            humidity.burn = 1;
            humidity.b_wink = 1; //启动闪烁
            core_get_tek_power()->on = 0;
            humidity.wink_tick = Get_Sys_Tick();
            humidity.wink_time = 0;
        }
        humidity.burn_pluse = 0;
    }
    
	#if 0
	if(humidity.power_off_delay)
    {
        if(Get_Sys_Tick() - power_off_delay_tick >= 60000)
        {
            humidity.power_off_delay = 0;
            core_get_tek_power()->on = 0;
            humidity.b_wink = 1; //启动闪烁
            humidity.wink_tick = Get_Sys_Tick();
            humidity.wink_time = 0;
        }
    }
	#endif
}

#else

tek_u8 drive_get_ks_pin(void);
tek_u8 drive_get_cs_pin(void);

static void _water_detect(void)
{
    static tek_u32  water_tick, cs_tick;
    static tek_u8  ks_hi_cnt, ks_low_cnt, cs_hi_cnt;
    if(Get_Sys_Tick() - water_tick >= 100)
    {
        water_tick = Get_Sys_Tick();
        if(drive_get_ks_pin())//高电平缺水
        {
            ks_hi_cnt++;
            ks_low_cnt = 0;
            if(ks_hi_cnt >= 10)
            {
                ks_hi_cnt = 0;
                humidity.low_water = 1;
                if(core_get_tek_power()->on)
                {
                    #if USE_LOW_WATER_DELAY_OFF
                    if(core_get_tek_power()->on_secs > 180) //开机3分钟后
                    {
                        humidity.power_off_delay = 1; //启动延时关机
                    }
                    else
                    {
                        core_get_tek_power()->on = 0;
                        humidity.b_wink = 1; //启动闪烁
                        humidity.wink_tick = Get_Sys_Tick();
                        humidity.wink_time = 0;
                    }
                    #else
                    core_get_tek_power()->on = 0;
                    humidity.b_wink = 1; //启动闪烁
                    humidity.wink_tick = Get_Sys_Tick();
                    humidity.wink_time = 0;
                    #endif
                }
            }
        }
        else
        {
            ks_hi_cnt = 0;
            ks_low_cnt++;
            if(ks_low_cnt >= 10)
            {
                ks_low_cnt = 0;
                humidity.low_water = 0;
                power_off_delay_tick = Get_Sys_Tick();
            }
        }
    }
    if(Get_Sys_Tick() - cs_tick >= 10)
    {
        cs_tick = Get_Sys_Tick();
        if(core_get_tek_power()->on)
        {
            if(!drive_get_cs_pin())//低电平干烧
            {
                cs_hi_cnt++;
                if(cs_hi_cnt >= 10)
                {
                    cs_hi_cnt = 0;
                    humidity.burn = 1;
                    core_get_tek_power()->on = 0;
                    humidity.b_wink = 1; //启动闪烁
                    humidity.wink_tick = Get_Sys_Tick();
                    humidity.wink_time = 0;
                }
            }
            else
            {
                cs_hi_cnt = 0;
            }
        }
        else
        {
            cs_hi_cnt = 0;
        }
    }
}

#endif

static void _core_lack_water_delay_power_off(void)
{
    if(humidity.power_off_delay)
    {
        if(Get_Sys_Tick() - power_off_delay_tick >= 60000)
        {
            humidity.power_off_delay = 0;
            core_get_tek_power()->on = 0;
            humidity.b_wink = 1; //启动闪烁
            humidity.wink_tick = Get_Sys_Tick();
            humidity.wink_time = 0;
        }
    }
}


#if USE_TE
static void _core_himidity_te_register(void)
{
    static tek_u8  te_register;
    if(!te_register)
    {
        te_register = 1;
        core_himidity_te_register(&humidity);
    }
}
#endif

#if USE_RH_RT
#define	CB_RTADCCNT				8				//热敏电阻采样次数
#define	CB_JIAQUAN				10				//每累加CB_RHCNT后减去CB_JIAQUAN组数据,因此CB_JIAQUAN不能大于CB_RHCNT
#define	CB_ADCBITS				(4096UL)    //ADC分辨率10位
#define	CB_RH_Resistor		(39UL)        //湿敏分压电阻39K
#define	CB_RT_Resistor		(10UL)				//热敏分压电阻10K
#define	CB_RHCNTTEMP			50				//RH于中断中采样次数,采样次数必须小于FFFF/3FF=64
#define	CB_TEMPADCCNT			50				//温度采样次数
#define	CB_RHCNT					50				//最大累计次数可达255
#define	CB_HENGSHIERROR		2					//恒湿允许的偏差值,即假定设定的恒湿值为50%，则湿度达到48%后重新加湿
#define CB_HUMSET					35				//湿度设置初始值

CONST_T tek_u16 CODE_T Code_RT[] = 					//热敏电阻参数表
{
    CB_ADCBITS*CB_RT_Resistor/(CB_RT_Resistor+552.9),			//-10
    CB_ADCBITS*CB_RT_Resistor/(CB_RT_Resistor+423.3),			//-5
    CB_ADCBITS*CB_RT_Resistor/(CB_RT_Resistor+325.1),			//0
    CB_ADCBITS*CB_RT_Resistor/(CB_RT_Resistor+255.7),			//5
    CB_ADCBITS*CB_RT_Resistor/(CB_RT_Resistor+201.4),			//10
    CB_ADCBITS*CB_RT_Resistor/(CB_RT_Resistor+159.19),			//15
    CB_ADCBITS*CB_RT_Resistor/(CB_RT_Resistor+126.48),			//20
    CB_ADCBITS*CB_RT_Resistor/(CB_RT_Resistor+100),				//25
    CB_ADCBITS*CB_RT_Resistor/(CB_RT_Resistor+81.94),			//30
    CB_ADCBITS*CB_RT_Resistor/(CB_RT_Resistor+66.52),			//35
    CB_ADCBITS*CB_RT_Resistor/(CB_RT_Resistor+53.26),			//40
    CB_ADCBITS*CB_RT_Resistor/(CB_RT_Resistor+43.66),			//45
};

CONST_T tek_u16 CODE_T Code_RH[] = 					//湿敏电阻参数表
{						
//---------------------------------------------------------------------------------------
//0℃
//---------------------------------------------------------------------------------------
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+4277),		//20%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+3645),		//25%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+3012),		//30%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2445),		//35%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1706),		//40%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1213),		//45%			
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+802),			//50%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+313),			//55%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+154),			//60%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+73),			//65%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+32),			//70%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+20),			//75%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+14.6),		//80%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+8.9),			//85%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+5.6),			//90%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+3.2),			//95%		
//---------------------------------------------------------------------------------------
//5℃
//---------------------------------------------------------------------------------------
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+3937),		//20%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+3317),		//25%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2833),		//30%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2088),		//35%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1416),		//40%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1011),		//45%			
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+489),			//50%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+208),			//55%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+111),			//60%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+52),			//65%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+25),			//70%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+18.3),		//75%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+11.3),		//80%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+6.9),			//85%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+4.2),			//90%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2.4),			//95%
//---------------------------------------------------------------------------------------
//10℃
//---------------------------------------------------------------------------------------
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+3645),		//20%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+3012),		//25%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2445),		//30%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1706),		//35%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1213),		//40%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+802),			//45%			
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+313),			//50%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+154),			//55%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+73),			//60%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+32),			//65%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+20),			//70%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+14.6),		//75%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+8.9),			//80%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+5.6),			//85%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+3.2),			//90%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2.1),			//95%
//---------------------------------------------------------------------------------------
//15℃
//---------------------------------------------------------------------------------------
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+3317),		//20%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2833),		//25%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2088),		//30%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1416),		//35%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1011),		//40%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+489),			//45%			
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+208),			//50%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+111),			//55%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+52),			//60%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+25),			//65%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+18.3),		//70%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+11.3),		//75%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+6.9),			//80%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+4.2),			//85%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2.4),			//90%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.9),			//95%
//---------------------------------------------------------------------------------------
//20℃
//---------------------------------------------------------------------------------------
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+3012),		//20%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2445),		//25%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1706),		//30%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1213),		//35%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+802),			//40%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+313),			//45%			
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+154),			//50%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+73),			//55%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+32),			//60%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+20),			//65%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+14.6),		//70%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+8.9),			//75%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+5.6),			//80%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+3.2),			//85%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2.1),			//90%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.8),			//95%
//---------------------------------------------------------------------------------------
//25℃
//---------------------------------------------------------------------------------------
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2833),		//20%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2088),		//25%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1416),		//30%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1011),		//35%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+489),			//40%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+208),			//45%			
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+111),			//50%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+52),			//55%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+25),			//60%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+18.3),		//65%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+11.3),		//70%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+6.9),			//75%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+4.2),			//80%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2.4),			//85%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.9),			//90%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.7),			//95%
//---------------------------------------------------------------------------------------
//30℃
//---------------------------------------------------------------------------------------
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2445),		//20%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1706),		//25%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1213),		//30%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+802),			//35%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+313),			//40%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+154),			//45%			
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+73),			//50%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+32),			//55%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+20),			//60%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+14.6),		//65%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+8.9),			//70%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+5.6),			//75%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+3.2),			//80%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2.1),			//85%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.8),			//90%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.4),			//95%
//---------------------------------------------------------------------------------------
//35℃
//---------------------------------------------------------------------------------------
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2088),		//20%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1416),		//25%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1011),		//30%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+489),			//35%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+208),			//40%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+111),			//45%			
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+52),			//50%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+25),			//55%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+18.3),		//60%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+11.3),		//65%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+6.9),			//70%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+4.2),			//75%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2.4),			//80%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.9),			//85%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.7),			//90%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.3),			//95%
//---------------------------------------------------------------------------------------
//40℃
//---------------------------------------------------------------------------------------
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1706),		//20%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1213),		//25%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+802),			//30%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+313),			//35%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+154),			//40%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+73),			//45%			
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+32),			//50%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+20),			//55%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+14.6),		//60%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+8.9),			//65%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+5.6),			//70%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+3.2),			//75%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2.1),			//80%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.8),			//85%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.4),			//90%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.2),			//95%
//---------------------------------------------------------------------------------------
//45℃
//---------------------------------------------------------------------------------------
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1416),		//20%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1011),		//25%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+489),			//30%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+208),			//35%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+111),			//40%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+52),			//45%			
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+25),			//50%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+18.3),		//55%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+11.3),		//60%		
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+6.9),			//65%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+4.2),			//70%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+2.4),			//75%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.9),			//80%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.7),			//85%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.3),			//90%
    CB_ADCBITS*CB_RH_Resistor/(CB_RH_Resistor+1.1),			//95%
    
    };

tek_u16 drive_GetADCResult(void);
tek_u16 drive_GetADCResultFromISR(void);

/******************************************************************************************************
* Funtion    : AdcA2T()
* Description: 从AD值到温度值
* Argument   : AdcA AD值
* Returns    : 温度值
*******************************************************************************************************/
static int _core_AdcA2T(tek_u32 AdcA,int *rtoff,int *rtstep)
{
	 int  t,B_i=0;
	 int  TempPoint = 0;
	
	
		for(B_i = 0; B_i < 10; B_i++)		//从0度开始查表
		{
			if(AdcA <= Code_RT[B_i + 2] || B_i == 9) break;
		}	
	
	  *rtstep = B_i;
		
	 if(AdcA>=Code_RT[11])
	 {
		 t=45;
		 *rtoff = TempPoint;
		 return t;
	 }
	 
	 if(AdcA<=Code_RT[0])
	 {
		 t=-10;
		 *rtoff = TempPoint;
		 return t;
	 }
 
	
	while( (B_i<12) && (Code_RT[B_i] < AdcA))
	{
    B_i++;
	}

  TempPoint = (AdcA-Code_RT[B_i-1])*5/(Code_RT[B_i]-Code_RT[B_i-1]);
	t=(B_i-1)*5+TempPoint -10;
	
	*rtoff = TempPoint;

	return t;
		
}

tek_bool b_get_ADCH;
static tek_bool  b_cal_hum;
static tek_u16  adch[8];
static tek_u8  adch_step;
tek_bool mutex;
tek_u16 W_RHTEMP;
static void	_core_humi_detect(void)
{
	tek_u16 B_i;
	static tek_u16  B_RHUP,B_RHDOWN,B_RHSTEP,W_RH;
	static int  B_RTXZ,B_RHXZ,B_RTSTEP=6;
	tek_u16 W_ADTEMP;
	

	static tek_u32  ADC_TEMP;
	static tek_u8  get_adc_temp_index,B_RHCNTTEMP;
	static tek_u32  temp_tick;
	static tek_u16	  B_HUMarray[30];
    static tek_u8   B_HUMindex,B_HUM_step;
	static tek_u32	 W_RHADD;											//湿度值累加

//---------------------------------------------------------------------------------------
//求取温度值及温度对应的档位值
//---------------------------------------------------------------------------------------
	if(Get_Sys_Tick() - temp_tick>1000 && !mutex)//1秒检测一次，温度检测完再打开湿度采样，这样湿度采样也同样1秒检测一次
	{  
		 static tek_bool  b_first;
		 temp_tick = Get_Sys_Tick();
//		
		 ADC_TEMP += drive_GetADCResult();
		 mutex=1;
		 get_adc_temp_index++;
		 if(!b_first)
		 {
			 b_first=1;
			 humidity.temperature = _core_AdcA2T(ADC_TEMP,&B_RTXZ,&B_RTSTEP);//上电第一次直接计算温度
		 }
		
		 if(get_adc_temp_index>=CB_RTADCCNT)
		 {
			 get_adc_temp_index=0;
			 W_ADTEMP = (ADC_TEMP/CB_RTADCCNT);	//求取平均值
			 ADC_TEMP=0;	 
			 humidity.temperature = _core_AdcA2T(W_ADTEMP,&B_RTXZ,&B_RTSTEP);
		 
		 }
		 #ifdef DEBUG_PRINT_LEVEL 
		 DEBUG_PRINT_D("temperature:%d B_RTXZ:%d  B_RTSTEP:%d\r\n",(int)humidity.temperature,(int)B_RTXZ,(int)B_RTSTEP);
         #endif
		
	}
	
	if(b_get_ADCH)
	{
		  tek_u32   sum;
		  b_get_ADCH=0;
		  #ifdef DEBUG_PRINT_LEVEL 
		  DEBUG_PRINT_D("adcH:%u\r\n",(unsigned int)W_RHTEMP);
          #endif
		  adch[B_RHCNTTEMP++]=W_RHTEMP;
			if(B_RHCNTTEMP>=8)B_RHCNTTEMP=0;
		
		  if(adch_step<8)adch_step++;
		
		  for(B_i=0,sum=0;B_i<adch_step;B_i++)
			{
				sum +=adch[B_i];
				
			}
			if(adch_step==0)return;
			W_RH=sum/adch_step;
			#ifdef DEBUG_PRINT_LEVEL 
			DEBUG_PRINT_D("ave adcH:%u\r\n",(unsigned int)W_RH);
            #endif
			
			b_cal_hum=1;
			
			
			
	}
	
	if(b_cal_hum)
	{
        b_cal_hum=0;
	
	//---------------------------------------------------------------------------------------
	//转存湿度采样值,同时判断湿度采样值是否合理,不合理说明线路故障
	//---------------------------------------------------------------------------------------		

		if(W_RH > Code_RH[(B_RTSTEP+1)*16-1])
		{
			W_RH = 	Code_RH[(B_RTSTEP+1)*16-1];//直接用最大值替代
		}		
//---------------------------------------------------------------------------------------
//求取温度下限对应的湿度值,同样的湿敏AD值,温度下限对应的湿度值高
//---------------------------------------------------------------------------------------	
		for(B_i = 0; B_i < 16; B_i++) 		//每个温度点对应16个湿度值
		{
			if(B_i == 15)					//若加到15进入for循环语句再+1为16,依此查表可能出错					
			{
				break;
			}
			else 
			{
				if(!B_RTSTEP)
				{
					if(W_RH <= Code_RH[B_i])
					{
						break;
					}
				}
				else
				{
			  		if(W_RH <= Code_RH[(B_RTSTEP-1)*16+B_i])//下限的最大值有可能小于实际采样值,需判断B_i == 15
					{
						break;
					}
				}
			}
		}
		B_RHSTEP = 	B_i;					//转移湿度档位
			
		if(!B_RHSTEP)
		{
			B_RHXZ = 0;						//已是最低档,修正值直接给0
		  B_RHDOWN = 20;
		}
		else 
		{
			if(!B_RTSTEP)
			{
				B_RHXZ = ((W_RH - Code_RH[B_RHSTEP - 1])*5)/(Code_RH[B_RHSTEP] - Code_RH[B_RHSTEP - 1]);//因每档湿度差值为5
			}
			else
			{
				if(W_RH > Code_RH[(B_RTSTEP-1)*16 + B_RHSTEP])
				{
					B_RHXZ = 5;				//采样值大于最大值,直接修正5	
				}
				else
				{
					B_RHXZ = ((W_RH - Code_RH[(B_RTSTEP-1)*16 + B_RHSTEP - 1])*5)/(Code_RH[(B_RTSTEP-1)*16 + B_RHSTEP] - Code_RH[(B_RTSTEP-1)*16 + B_RHSTEP - 1]);//因每档湿度差值为5
				}
			}
			B_RHDOWN = (B_RHSTEP - 1)*5 + 20 + B_RHXZ;//每隔差值为5,基准是20%
		}
//---------------------------------------------------------------------------------------
//求取温度上限对应的湿度值,同样的湿敏AD值,温度上限对应的湿度值低
//---------------------------------------------------------------------------------------
		for(B_i = 0; B_i < 16; B_i++) 		//每个温度点对应16个湿度值
		{
			if(B_i == 15)					//若加到15进入for循环语句再+1为16,依此查表可能出错					
			{
				break;
			}
			else if(W_RH <= Code_RH[B_RTSTEP*16+B_i])
			{
				break;
			}
		}
		
		B_RHSTEP = 	B_i;					//转移湿度档位
		if(!B_RHSTEP)
		{
			B_RHXZ = 0;						//已是最低档,修正值直接给0
			B_RHUP = 20;					//每隔差值为5,基准是20%
		}
		else 
		{
			B_RHXZ = ((W_RH - Code_RH[B_RTSTEP*16 + B_RHSTEP - 1])*5)/(Code_RH[B_RTSTEP*16 + B_RHSTEP] - Code_RH[B_RTSTEP*16 + B_RHSTEP - 1]);//因每档湿度差值为5
			B_RHUP = (B_RHSTEP - 1)*5 + 20 + B_RHXZ;//每隔差值为5,基准是20%
		}	
//---------------------------------------------------------------------------------------
//结合湿度上限、下限及温度修正值求取最终的湿度值,并累计N次后求平均值					
//---------------------------------------------------------------------------------------
		B_HUMarray[B_HUMindex++]= (B_RHDOWN - ((B_RHDOWN - B_RHUP)/5)*B_RTXZ);
		if(B_HUMindex>29)B_HUMindex=0;
		if(B_HUM_step<30)B_HUM_step++;
		
		for(B_i = 0,W_RHADD=0; B_i < B_HUM_step; B_i++)
		{
			
			  W_RHADD +=B_HUMarray[B_i];
		}
		
		humidity.humidity =W_RHADD/B_HUM_step;
        #ifdef DEBUG_PRINT_LEVEL 
		DEBUG_PRINT_D("humidity:%d\r\n",(int)humidity.humidity);
        #endif
	}
}

#endif


void core_humidity_handle(void)
{
    #if USE_TE
    _core_himidity_te_register();
    #endif
    _water_detect();
    _core_lack_water_delay_power_off();

    #if USE_RH_RT
    _core_humi_detect();
    #endif
}


#endif




