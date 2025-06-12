
#include "ihastek.h"


#if USE_BUZZER
static tek_u8  buzz_times, b_buzz;
static tek_u32  buzz_tick;


void drive_buzz_open(void);
void drive_buzz_close(void);




#if USE_TE
void core_buzzer_on_off(tek_bool onoff)
{
    if(onoff) { drive_buzz_open(); }
    else { drive_buzz_close(); }
}
#endif

//static TEKBuzzer_t  tek_buzz;

void core_set_buzzer(tek_u8 times)
{
//    if(core_get_tek_power()->lost) { return; }
    if(times == 0) { return; }
    buzz_times = times;
    buzz_tick = Get_Sys_Tick();
    if(buzz_times)
    {
        drive_buzz_open();
        b_buzz = 1;
    }
}


void core_buzz_drive(void)
{
    #if USE_TE
    static tek_bool is_buzz_register;
    if(!is_buzz_register)
    {
        is_buzz_register = 1;
        tek_buzz.fre = BUZZER_FRE;
        core_buz_te_register(&tek_buzz);
    }
    #endif
    if(buzz_times)
    {
        if(buzz_times>5)
        {
            buzz_times = 0;
            printf("buzz_times>5 error!!!\r\n");
            return;
        }
        if(Get_Sys_Tick() - buzz_tick > 70)
        {
            buzz_tick = Get_Sys_Tick();
            if(b_buzz)
            {
                drive_buzz_close();
                b_buzz = 0;
                buzz_times--;
            }
            else
            {
                drive_buzz_open();
                b_buzz = 1;
            }
        }
    }
}
#endif


#if USE_CHORD_MUSIC

void drive_Buzz_Disable(void);
void drive_Buzz_PWM(tek_u32 fre);
void drive_set_buz_on(tek_u8 onoff);

Buzz_TypeDef  Buzz;

void core_set_buzzer(CHORD_ENUM type)
{
    /*if(core_get_tek_power()->lost == 0)*/ { Buzz.Type = type; }
}


static void delay_ms(void)
{
    volatile tek_u16 i, j;
    for(i = 0; i < 10; i++)
    {
        for(j = 0; j < 100; j++);
    }
}

#if USE_TE
void core_buzzer_on_off(tek_bool onoff)
{
    if(onoff)
    {
        core_set_buzzer(chord_SD);
    }
}
#endif

void core_buzz_drive(void)
{
    static tek_bool  buz_disable;
    //    static tek_bool is_buzz_register;
    //    if(!is_buzz_register)
    //    {
    //        is_buzz_register = 1;
    //        core_buz_te_register(&tek_buzz);
    //    }
    //---------------------------------------------------------------------------------------
    //初始化
    //---------------------------------------------------------------------------------------
    if(Buzz.Type & 0x80)
    {
        #ifdef DEBUG_PRINT_LEVEL
        DEBUG_PRINT_D("BUZZ init\r\n");
        #endif
        Buzz.Type &= 0x7f;
        Buzz.Start = 1;
        Buzz.StartTime = 0;
        Buzz.Time = 0;
        Buzz.Step = 0;
        Buzz.En = 0;
        drive_Buzz_Disable();
        delay_ms();
        drive_set_buz_on(1);
    }
    if(Buzz.Start && (Get_Sys_Tick() - Buzz.TimerBase10ms >= 10))
    {
        Buzz.TimerBase10ms = Get_Sys_Tick();
        Buzz.StartTime++;
        if(Buzz.StartTime == CB_buzz_start_time)
        {
            Buzz.StartTime = 0;
            Buzz.Start = 0;
            Buzz.En = 1;
            drive_set_buz_on(0);
            if((Buzz.Type | 0x80) == chord_KEY)
            {
                drive_Buzz_PWM(1538);
            }
        }
    }
    //---------------------------------------------------------------------------------------
    //650US = 1538Hz
    //---------------------------------------------------------------------------------------
    if(Buzz.En && Get_Sys_Tick() - Buzz.TimerBase100ms >= 100)
    {
        Buzz.TimerBase100ms = Get_Sys_Tick();        //100MS
        Buzz.Time++;
        if((Buzz.Type | 0x80) == chord_KEY)
        {
            if(Buzz.Time >= CB_buzz_time_key)
            {
                Buzz.Time = 0;       
                Buzz.En = 0;       
            }
            else if(Buzz.Time == 1)
            {
                drive_Buzz_PWM(1538); //1538
            }
        }

        else if((Buzz.Type | 0x80) == chord_KJ)
        {
            switch(Buzz.Step)
            {
                case 0:
                    if(Buzz.Time >= CB_buzz_time_kj1)
                    {
                        Buzz.Time = 0;     
                        Buzz.Step++;
                        drive_Buzz_PWM(1000); 
                        Buzz.Start = 1;     
                        drive_set_buz_on(1);
                    }
                    else if(Buzz.Time == 1)
                    {
                        drive_Buzz_PWM(1538); //1538 
                    }
                    break;
                case 1:
                    if(Buzz.Time >= CB_buzz_time_kj2)
                    {
                        Buzz.Time = 0;
                        Buzz.En = 0;
                    }
                    break;
                default:
                    break;
            }
        }
        else
        {
            switch(Buzz.Step)
            {
                case 0:
                    if(Buzz.Time == CB_buzz_time_sd1)
                    {
                        Buzz.Time = 0;     
                        Buzz.Step++;
                        drive_Buzz_PWM(1250); //1250   
                        Buzz.Start = 1;      
                        drive_set_buz_on(1);
                    }
                    else if(Buzz.Time == 1)
                    {
                        drive_Buzz_PWM(1000); //1000     
                    }
                    break;
                case 1:
                    if(Buzz.Time == CB_buzz_time_sd2)
                    {
                        Buzz.Time = 0;               
                        Buzz.Step++;
                        drive_Buzz_PWM(1428);     //1428     
                        Buzz.Start = 1;      //
                        drive_set_buz_on(1);
                    }
                    break;
                case 2:
                    if(Buzz.Time == CB_buzz_time_sd3)
                    {
                        Buzz.Time = 0;              
                        Buzz.Step++;
                        drive_Buzz_PWM(2000);     //2000        
                        Buzz.Start = 1;      //
                        drive_set_buz_on(1);
                    }
                    break;
                case 3:
                    if(Buzz.Time == CB_buzz_time_sd4)
                    {
                        Buzz.Time = 0;                
                        Buzz.Step++;
                        drive_Buzz_PWM(2500);     //2500        
                        Buzz.Start = 1;     
                        drive_set_buz_on(1);
                    }
                    break;
                case 4:
                    if(Buzz.Time == CB_buzz_time_sd5)
                    {
                        Buzz.Time = 0;          
                        Buzz.En = 0;        
                    }
                    break;
                default:
                    break;
            }
        }
    }
    if(!Buzz.En)
    {
        if(buz_disable) { drive_Buzz_Disable(); }
        buz_disable = 0;
    }
    else
    {
        buz_disable = 1;
    }
}

#endif

