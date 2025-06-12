
#include "ihastek.h"


#if USE_FAN_PWM
static TEKFan_t fan;
static TEKPWM_t tek_pwm;

TEKPWM_t *core_tek_GetPWM(void)
{
 return &tek_pwm;
}


extern const unsigned int speed_rpm[];

void core_rpm_table_register(TEKSpeed_t *tek_speed)
{
    fan.rpm_table = tek_speed->table;
    fan.step_all = tek_speed->size;
    tek_pwm.max = tek_speed->max;
    tek_pwm.fre = tek_speed->fre;
}
#if(USE_FAN_FG)


static TEKFG_t FG;
unsigned int fg_cn_tem;
extern volatile unsigned int FG_FeedBackCircluNum;

#if USE_TE
static void _core_te_register(void)
{
    static bool te_register;
    if(te_register == false)
    {
        te_register = true;
        core_pwm_te_register(&tek_pwm);
//        #if USE_FG
//        core_fg_te_register((TEKFG_t *)&FG);
//        #endif
    }
}
#endif
//																						1		2		3		4		5		6		7		8		9		10
const uint16_t FanDutyArray[SPEEDLEVEL_MAX]={700,660,615,510,440,375,315,250,179,118};//{790,734,678,622,566,510,454,398,342,284};
typedef struct{
	uint16_t rpm;
	uint16_t duty;
}TEKTEFAN_t;

void core_speed_handle(void)
{
	static unsigned int fan_speed_duty_last;
	static unsigned char fan_speed_old;
	static bool preonoffstatus = 0;
	unsigned char pulse = 15;
//	static bool aa=false;
	#if USE_TE
	static unsigned int te_set_rpm,prete_set_rpm;
	_core_te_register();
	#endif
	
	
//	if(aa == false)
//	{
//		aa = true;
//	tek_pwm.te_set=1;
//	te_set_rpm=1000;
//	}
	if(tek_pwm.te_set)
	{
		if(Get_Sys_Tick() - FG.tick >= 1000)
		{			
			FG.tick = Get_Sys_Tick();
			FG.cnt = fg_cn_tem;
			fg_cn_tem = 0;
			FAN_FG_PLUSE_PER_CIRCLE ? (pulse = FAN_FG_PLUSE_PER_CIRCLE) : (pulse = 15);
			core_get_tek_power()->rpmofspeed = (FG.cnt * 60) / pulse;
		}
		return;
	}
	else
	{
		if(Get_Sys_Tick() - FG.tick >= 1000)
		{			
			FG.tick = Get_Sys_Tick();
			FG.cnt = fg_cn_tem;
			fg_cn_tem = 0;
			FAN_FG_PLUSE_PER_CIRCLE ? (pulse = FAN_FG_PLUSE_PER_CIRCLE) : (pulse = 15);
			core_get_tek_power()->rpmofspeed = (FG.cnt * 60) / pulse;
		}
		if(core_get_tek_power()->on)
		{
				core_get_tek_power()->real_speed = core_get_tek_power()->speed.speedpara[core_get_tek_power()->speed.changetype].lev;
		}
		else
		{
				core_get_tek_power()->real_speed = 0;
				tek_pwm.duty = FAN_PWM_DUTY_MAX;
		}
	}
	
	if(tek_pwm.te_set != 1)
	{
		if((fan_speed_old != core_get_tek_power()->real_speed) || (preonoffstatus != core_get_tek_power()->on))
		{
			preonoffstatus = core_get_tek_power()->on;
				fan_speed_old = core_get_tek_power()->real_speed;
				tek_pwm.te_set =0;
			
				tek_pwm.duty = FanDutyArray[core_get_tek_power()->real_speed];//fan.rpm_table[core_get_tek_power()->real_speed];
				FG.tick = Get_Sys_Tick();
		}	
	}
    
	if(tek_pwm.te_set || core_get_tek_power()->on)
	{
		if(Get_Sys_Tick() - FG.tick >= 1000)
		{
			//unsigned int W_i;
			signed int rpm_diss;
			
			FG.tick = Get_Sys_Tick();
			FG.cnt = fg_cn_tem;
			fg_cn_tem = 0;
			FAN_FG_PLUSE_PER_CIRCLE ? (pulse = FAN_FG_PLUSE_PER_CIRCLE) : (pulse = 15);
			core_get_tek_power()->rpmofspeed = (FG.cnt * 60) / pulse;
			
            rpm_diss = (signed int)core_get_tek_power()->rpmofspeed-fan.rpm_table[core_get_tek_power()->real_speed+1];

			if(abs1(rpm_diss) >= FAN_FG_RPM_TOLERATION)
			{
				if(rpm_diss > 0)
					tek_pwm.duty++;
				else
					tek_pwm.duty--;
			}
			if(!fan.rpm_table)
			{
					return;
			}
		}
		
	}
	if((fan_speed_duty_last != tek_pwm.duty) && (!tek_pwm.te_set) )
	{
			fan_speed_duty_last = tek_pwm.duty;
			drive_pwm_duty_set(tek_pwm.duty);
	}
}

#else
void core_speed_handle(void)
{
    static unsigned int fan_speed_duty_last;
		static unsigned char fan_speed_old;
    #if USE_PWM_ADD_SLOWLY
    static unsigned long duty_tick;
    #endif
	
    #if USE_TE
    //_core_te_register();
    #endif
    #if USE_ZERO
    core_cal_zero();
    #endif
    if(core_get_tek_power()->on)
    {
		if(fan_speed_old != core_get_tek_power()->real_speed)
		{
				fan_speed_old = core_get_tek_power()->real_speed;
				tek_pwm.te_set =0;
		}		
		
		
        #if USE_PWM_ADD_SLOWLY
        if(tek_pwm.duty < (fan.rpm_table[1] * 2 / 3))
        {
            tek_pwm.duty = fan.rpm_table[1] * 2 / 3;
        }
        #else
        tek_pwm.duty = fan.rpm_table[core_get_tek_power()->real_speed];
        #endif
    }
    else
    {
        core_get_tek_power()->real_speed = 0;
        #if USE_PWM_ADD_SLOWLY
        if(tek_pwm.duty < (fan.rpm_table[1]))
        {
            tek_pwm.duty = FAN_PWM_DUTY_MAX;
        }
        #else
        tek_pwm.duty = FAN_PWM_DUTY_MAX;
        #endif
    }
    if(core_get_tek_power()->real_speed >= fan.step_all)
    {
        #if DEBUG_PRINT_ENABLE
        printf("speed(%u) >= fan step all(%u) ERROR.\r\n", (unsigned int)core_get_tek_power()->real_speed, (unsigned int)fan.step_all);
        #endif
        return;
    }
    #if USE_PWM_ADD_SLOWLY
    tek_pwm.target_duty = fan.rpm_table[core_get_tek_power()->real_speed];
		if(!tek_pwm.te_set)
		{
				if(sys_tick_get() - duty_tick > 50)
				{
						duty_tick = sys_tick_get();
						if(tek_pwm.target_duty > tek_pwm.duty)
						{
								tek_pwm.duty += (CB_PWMMAX / 200 + 1);
						}
						if(tek_pwm.target_duty < tek_pwm.duty)
						{
								tek_pwm.duty -= (CB_PWMMAX / 200 + 1);
						}
				}			
			
		}

    #endif
    if((fan_speed_duty_last != tek_pwm.duty) && (!tek_pwm.te_set) )
    {
        fan_speed_duty_last = tek_pwm.duty;
        drive_pwm_duty_set(tek_pwm.duty);
    }
}

#endif
#endif

