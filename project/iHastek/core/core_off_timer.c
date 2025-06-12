
#include "ihastek.h"


#if USE_TIMER


//static unsigned char  *time_table;
static unsigned char  time_size=13;
static TEKTime_t  off_timer;
TEKTime_t *core_get_tek_timer(void)
{
    return &off_timer;
}
const unsigned int time_table[] = {0,15,30,45,60,120,180,240,300,360,420,480,540};
//void core_off_timer_table_register(unsigned char *table, unsigned char table_size)
//{
//    time_table = table;
//    time_size =  table_size;
//}

void core_auto_off_handle(void)
{
    unsigned long run_seconds;
    #if USE_TE
    static bool  b_register;
    if(!b_register)
    {
        b_register = 1;
        core_off_timer_te_register(&off_timer);//关联TE
    }
    #endif
    if(off_timer.step >= time_size)
    {
        return;
    }
    if(off_timer.step_last != off_timer.step)
    {
        off_timer.step_last = off_timer.step;
        off_timer.hours = time_table[off_timer.step];
        off_timer.left_secs = (unsigned long)time_table[off_timer.step] * (unsigned long)AUTO_OFF_STEP_SECONDS;
        off_timer.all_secs = off_timer.left_secs;
        off_timer.start_tick = Get_Sys_Tick();
        off_timer.onesec_tick = Get_Sys_Tick();
    }
    if(off_timer.step)
    {
        if(Get_Sys_Tick() - off_timer.onesec_tick >= 1000)
        {
            off_timer.onesec_tick = Get_Sys_Tick();
            run_seconds = (Get_Sys_Tick() - off_timer.start_tick) / 1000;
            if(off_timer.all_secs > run_seconds)
            {
                off_timer.left_secs = off_timer.all_secs - run_seconds;
            }
            else
            {
                off_timer.left_secs = 0;
            }
            if(off_timer.left_secs <= (unsigned long)time_table[off_timer.step - 1] * (unsigned long)AUTO_OFF_STEP_SECONDS)
            {
                off_timer.step--;
            }
            if(off_timer.left_secs == 0)
            {
                platform_auto_off();
              
            }
        }
    }
    else
    {
        off_timer.left_secs = 0;
    }
    if(!core_get_tek_power()->on)
    {
        off_timer.step = 0;
        off_timer.left_secs = 0;
    }
}


/*

step:   设置定时档柆  
is_self_increase:是否内部自加一档,当为1时，第一个参数step无效
*/
void core_set_auto_off_step(unsigned char step, bool is_self_increase)//第一个参数：设置定时档柆  第二个参数：是否内部自加一档,当为1时，第一个参数无效
{
    if(is_self_increase)
    {
        off_timer.step++;
        off_timer.step %= time_size;
    }
    else
    {
        off_timer.step = step;
    }
    if(off_timer.step == 0) { off_timer.left_secs = 0; }
}

unsigned char core_get_auto_off_step(void)
{
    return   off_timer.step;
}

#endif
