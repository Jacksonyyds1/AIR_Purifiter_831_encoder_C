
#include "ihastek.h"

#if  USE_PM25

static TEKPM25_t PM25, PM10;
static const TEK_PM25_AUTO_SPEED_t *Code_PM25;

TEKPM25_t *core_get_pm25(void)
{
    return  &PM25;
}

//PM2.5档位判断
static void __core_pm25_step_handle(void)
{
    register uint8_t aqi_level;
    static uint32_t step_tick;
    if(!core_get_tek_power()->on)
    {
        return;
    }
    if((Get_Sys_Tick() - step_tick >= 1000))
    {
        step_tick = Get_Sys_Tick();
        aqi_level = PM25.aqi_level;
        if(PM25.aqi_level < PM25.aqi_level_size - 1)
        {
            if(PM25.num >= Code_PM25[aqi_level + 1].pm25)
            {
                aqi_level++;
            }
        }
        if(PM25.aqi_level >= 1)
        {
            if(PM25.num < (Code_PM25[aqi_level].pm25 - PM25_OFF_STEP))
            {
                aqi_level--;
            }
        }

        PM25.aqi_level = aqi_level;
        core_get_tek_power()->auto_speed = Code_PM25[aqi_level].auto_speed;
    }
}

#if USE_TE
static void __core_PM25_te_register(void)
{
    static bool PM25_register;
    if(!PM25_register)
    {
        PM25_register = 1;
        core_PM25_te_register(&PM25);
    }
}
#endif

void core_PM25_Main(void)
{

#if USE_TE
    __core_PM25_te_register();
#endif
    if(core_get_tek_power()->on)
    {
#if PM25SENSORSEL == USE_DSCD01 || USE_MPM12BG
        drive_set_pm25_on(1);
#endif
        __core_pm25_step_handle();
#if PM25SENSORSEL == USE_DSCD01
        if(Get_Sys_Tick() - pw25_tick >= 1000)
        {
            pw25_tick = Get_Sys_Tick();
            drive_PM25_uart_enable(1);
            //core_pm25_send();
        }
#endif
    }
#if PM25SENSORSEL == USE_DSCD01 || USE_MPM12BG
    else
    {
        drive_set_pm25_on(0);
    }
#endif
}

void core_pm25_table_register(const TEK_PM25_AUTO_SPEED_t *table, uint8_t size)
{
    Code_PM25 = table;
    PM25.aqi_level_size = size;
}

#endif