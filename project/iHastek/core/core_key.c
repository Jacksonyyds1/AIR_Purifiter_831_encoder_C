#include "ihastek.h"
#include "user_config.h"


#if USE_KEY

#if USE_KEY_TOUCH
extern void core_key_touch(uint16_t key);
#endif

#ifdef KEY_LONG_PRESS_TIME
extern void core_key_long_press(uint16_t key);
#endif

#if KEY_SECOND_LONG_PRESS_TIME//5s
extern void core_key_second_long_press(uint16_t key);
#endif

#if KEY_THIRDLY_LONG_PRESS//10s
extern void core_key_thirdly_long_press(uint16_t key);
#endif

static TEKKey_t key;
static void _core_ScanKey_main(void)
{
	static unsigned long keyScanTick;
	static uint8_t  B_KEY_ON, B_KEY_OFF;
    
	if(Get_Sys_Tick()-keyScanTick >= 10)
	{
		keyScanTick=Get_Sys_Tick();
		
    key.now = core_get_key_value();
		
    if(key.now)
    {
        if(key.now == key.last)
        {
            B_KEY_OFF = 0;
            B_KEY_ON++;
            if(B_KEY_ON >= KEY_DEBOUNCING)
            {
                B_KEY_ON = 0;
                key.te = key.now;
                key.ok = 1;
            }
        }
        else
        {
            key.last = key.now;
            B_KEY_ON = 0;
            key.keep_tick = Get_Sys_Tick();
            key.keep_thirdly_tick = Get_Sys_Tick();

        }
    }
    else
    {
        B_KEY_ON = 0;
        B_KEY_OFF++;
        if(B_KEY_OFF >= KEY_DEBOUNCING)
        {
            B_KEY_OFF = 0;
#if USE_KEY_RELEASE
            if(/*key.keep || */key.second_keep)//������Ч������ʱ������
            {
                key.up = 0;
            }
            if(key.up)
            {
							if(Get_Sys_Tick() - core_get_tek_power()->DSModeTick > 300)
                core_key_release(key.verify);
                // core_key_release(key.now);
                key.delay_release = 1;
            }
#endif
            key.up = 0;
            key.keep = 0;
            key.second_keep = 0;
            key.third_keep = 0;
            key.last = 0;
            key.verify = 0;
            key.keep_tick = Get_Sys_Tick();
            key.keep_thirdly_tick = Get_Sys_Tick();
			key.KeyHoldAfterLP.LPType = key.KeyHoldAfterLP.Status = key.KeyHoldAfterLP.KeyValue = 0;
        }
    }

	}
}

static void _core_KEY_main(void)
{
    if(key.ok)
    {
        key.ok = 0;
        if(key.verify == key.last)
        {
#ifdef KEY_LONG_PRESS_TIME
            if(Get_Sys_Tick() - key.keep_tick >= KEY_LONG_PRESS_TIME)
            {
                if(!key.keep)
                {
                    key.keep = 1;
                   //key.up = 0;
                    core_key_long_press(key.verify);
                }
            }
#endif
#if  KEY_SECOND_LONG_PRESS_TIME
            if(Get_Sys_Tick() - key.keep_tick >= KEY_SECOND_LONG_PRESS_TIME)
            {
                if(!key.second_keep)
                {
                    key.second_keep = 1;
									if(KEY_POWER != key.verify)
                    key.up = 0;
                    core_key_second_long_press(key.verify);
                }
            }
#endif
#if KEY_THIRDLY_LONG_PRESS
			if(Get_Sys_Tick() - key.keep_tick >= KEY_THIRDLY_LONG_PRESS)
            {
                if(!key.third_keep)
                {
                    key.third_keep = 1;
                    key.up = 0;
                    core_key_thirdly_long_press(key.verify);
                }
            }
#endif
        }
        else
        {
            key.verify = key.last;
#if USE_KEY_TOUCH
            core_key_touch(key.verify);
#endif
            key.up = 1;
            key.once_up = 1;
        }
        key.once_up_tick = Get_Sys_Tick();
    }
#if USE_ONCE_UP_KEY
    if(core_get_tek_power()->on == 0)
    {
        key.once_up = 0;
        key.once_up_tick = Get_Sys_Tick();
    }
#endif
}



void core_KEY_main(void)
{
#if USE_TE
    static bool  key_register;
    if(!key_register)
    {
        key_register = 1;
        core_key_te_register(&key);
			
    }
#endif
    _core_ScanKey_main();
    _core_KEY_main();
}
#endif
