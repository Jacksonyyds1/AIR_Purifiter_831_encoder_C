
#include "ihastek.h"


#ifdef HEPA_TIMEOUT
static TEKHepa_t  tek_hepa;


#ifdef HEPA2_TIMEOUT
    static   TEKHepa_t  tek_hepa2;
#endif

#ifdef HEPA3_TIMEOUT
    static  TEKHepa_t  tek_hepa3;
#endif

static TEKFlashWrite_t   tek_flash_write_t;

TEKHepa_t *core_get_hepa(void)
{
    return &tek_hepa;
}

void core_hepa_reset(void)
{
    tek_hepa.reset = 1;
    tek_hepa.flash_tick = Get_Sys_Tick();
    tek_hepa.flash_time = 3;
}
#if 1
void core_flash_write(TEKFlash_e type)
{
    unsigned char *pflash_write=NULL;
    switch(type)
    {
        case TEK_ENUM_HEPA:
            pflash_write = (unsigned char *)&tek_flash_write_t.hepa;
            break;
				#ifdef HEPA2_TIMEOUT
        case TEK_ENUM_HEPA2:
            pflash_write = (unsigned char *)&tek_flash_write_t.hepa2;
            break;
				#endif
				#ifdef HEPA3_TIMEOUT
        case TEK_ENUM_HEPA3:
            pflash_write = (unsigned char *)&tek_flash_write_t.hepa3;
            break;
				#endif
//        case TEK_ENUM_ADDR:
//            pflash_write = (unsigned char *)&tek_flash_write_t.addr;
//            break;
//        case TEK_ENUM_RESERVE:
//            pflash_write = (unsigned char *)&tek_flash_write_t.reserve;
//            break;
        default:
            break;
    }
    if(NULL!=pflash_write)*pflash_write = 1;
}
#endif
#if USE_TE
void core_te_set_hepa(unsigned long hepa_s)
{
    tek_hepa.secs = hepa_s;
    tek_hepa.start_tick = Get_Sys_Tick();
    tek_hepa.start_secs = tek_hepa.secs;
    tek_hepa.start_all_secs = tek_hepa.all_secs;
    tek_hepa.human = 1;
    core_flash_write(TEK_ENUM_HEPA);
}
#endif


void core_hepa_fast_test(void)
{
    tek_hepa.secs = HEPA_TIMEOUT - 3600UL;
    tek_hepa.start_tick = Get_Sys_Tick();
    tek_hepa.start_secs = tek_hepa.secs;
    tek_hepa.start_all_secs = tek_hepa.all_secs;
    tek_hepa.human = 1;
    core_flash_write(TEK_ENUM_HEPA);
}

#ifdef HEPA2_TIMEOUT

TEKHepa_t *core_get_hepa2(void)
{
    return &tek_hepa2;
}
void core_hepa2_reset(void)
{
    tek_hepa2.reset = 1;
    tek_hepa2.flash_tick = Get_Sys_Tick();
    tek_hepa2.flash_time = 3;	
}

#if USE_TE
void core_te_set_hepa2(unsigned long hepa_s)
{
    tek_hepa2.secs = hepa_s;
    tek_hepa2.start_tick = Get_Sys_Tick();
    tek_hepa2.start_secs = tek_hepa2.secs;
    tek_hepa2.start_all_secs = tek_hepa2.all_secs;
    tek_hepa2.human = 1;
    core_flash_write(TEK_ENUM_HEPA2);
}
#endif

void core_hepa2_fast_test(void)
{
    tek_hepa2.secs = HEPA2_TIMEOUT - 3600UL;
    tek_hepa2.start_tick = Get_Sys_Tick();
    tek_hepa2.start_secs = tek_hepa2.secs;
    tek_hepa2.start_all_secs = tek_hepa2.all_secs;
    tek_hepa2.human = 1;
    core_flash_write(TEK_ENUM_HEPA2);
}
#endif  //HEPA2_TIMEOUT


#ifdef HEPA3_TIMEOUT

TEKHepa_t *core_get_hepa3(void)
{
    return &tek_hepa3;
}

void core_hepa3_reset(void)
{
    tek_hepa3.reset = 1;
    tek_hepa3.flash_tick = Get_Sys_Tick();
    tek_hepa3.flash_time = 3;	
}

#if USE_TE
void core_te_set_hepa3(unsigned long hepa_s)
{
    tek_hepa3.secs = hepa_s;
    tek_hepa3.start_tick = Get_Sys_Tick();
    tek_hepa3.start_secs = tek_hepa3.secs;
    tek_hepa3.start_all_secs = tek_hepa3.all_secs;
    tek_hepa3.human = 1;
    core_flash_write(TEK_ENUM_HEPA3);
}
#endif


void core_hepa3_fast_test(void)
{
    tek_hepa3.secs = HEPA3_TIMEOUT - 3600UL;
    tek_hepa3.start_tick = Get_Sys_Tick();
    tek_hepa3.start_secs = tek_hepa3.secs;
    tek_hepa3.start_all_secs = tek_hepa3.all_secs;
    tek_hepa3.human = 1;
    core_flash_write(TEK_ENUM_HEPA3);
}
#endif  //HEPA3_TIMEOUT
#if 0
void core_flash_write(TEKFlash_e type)
{
    static unsigned long  detect_tick5[5] = {0}, detect_tick3600[5] = {0};
    static unsigned long  write_times5[5] = {0}, write_times3600[5] = {0};
    static unsigned char  No_write[5] = {0};
    unsigned char *pflash_write;
    unsigned long *pdetect_tick5;
    unsigned long *pdetect_tick3600;
    unsigned long *pwrite_times5;
    unsigned long *pwrite_times3600;
    unsigned char *pNo_write;
    switch(type)
    {
        case TEK_ENUM_HEPA:
            pflash_write = (unsigned char *)&tek_flash_write_t.hepa;
            break;
        case TEK_ENUM_HEPA2:
            pflash_write = (unsigned char *)&tek_flash_write_t.hepa2;
            break;
        case TEK_ENUM_HEPA3:
            pflash_write = (unsigned char *)&tek_flash_write_t.hepa3;
            break;
        case TEK_ENUM_ADDR:
            pflash_write = (unsigned char *)&tek_flash_write_t.addr;
            break;
        case TEK_ENUM_RESERVE:
            pflash_write = (unsigned char *)&tek_flash_write_t.reserve;
            break;
        default:
            return;
            break;
    }
    pdetect_tick5 = (unsigned long *)&detect_tick5[type];
    pdetect_tick3600 = (unsigned long *)&detect_tick3600[type];
    pwrite_times5 = (unsigned long *)&write_times5[type];
    pwrite_times3600 = (unsigned long *)&write_times3600[type];
    pNo_write = (unsigned char *)&No_write[type];
    //5����д�Ĵ�������5��д�Ĵ�������3�Σ���ر�д�Ĺ���
    *pwrite_times5 = (*pwrite_times5) + 1 ;
    if(Get_Sys_Tick() - *pdetect_tick5 > 5000)
    {
        *pdetect_tick5 = Get_Sys_Tick();
        if(*pwrite_times5 > 3)
        {
            *pNo_write = 1;
            #ifdef DEBUG_PRINT_LEVEL
            DEBUG_PRINT_E("Flash write fast ERROR!!!\r\n");
            #endif
        }
        *pwrite_times5 = 0;
    }
    //3600����д�Ĵ�������3600��д�Ĵ�������50�Σ���ر�д�Ĺ���
    *pwrite_times3600 = (*pwrite_times3600) + 1;
    if(Get_Sys_Tick() - *pdetect_tick3600 > 3600000)
    {
        *pdetect_tick3600 = Get_Sys_Tick();
        if(*pwrite_times3600 > 50)
        {
            *pNo_write = 1;
            #ifdef DEBUG_PRINT_LEVEL
            DEBUG_PRINT_E("Flash write fast ERROR!!!\r\n");
            #endif
        }
        *pwrite_times3600 = 0;
    }
    if(*pNo_write)
    {
        //�ر�д�Ĺ��ܺ�ÿСʱ��ͨдһ��
        static unsigned long  No_write_tick;
        if(Get_Sys_Tick() - No_write_tick >= 3600000)
        {
            No_write_tick = Get_Sys_Tick();
            if(core_get_tek_power()->on) { *pflash_write = 1; }
        }
    }
    else
    {
        *pflash_write = 1;
    }
		
		
		
}
#endif





static void __core_heap_write(void)
{
    core_flash_write(TEK_ENUM_HEPA);
    #ifdef HEPA2_TIMEOUT
    core_flash_write(TEK_ENUM_HEPA2);
    #endif
    #ifdef HEPA3_TIMEOUT
    core_flash_write(TEK_ENUM_HEPA3);
    #endif
}



static void __core_hepa_data(TEKFlash_e type, TEKHepa_t hepa)
{
    unsigned char  B_EEPROMTEMP[16] = {0}, i;
    unsigned char sumcal = 0;
    B_EEPROMTEMP[0] = HEPA_WRITE_FLAG;
    B_EEPROMTEMP[1] = hepa.secs & 0XFF;
    B_EEPROMTEMP[2] = (hepa.secs >> 8) & 0XFF;
    B_EEPROMTEMP[3] = (hepa.secs >> 16) & 0XFF;
    B_EEPROMTEMP[4] = (hepa.secs >> 24) & 0XFF;
    B_EEPROMTEMP[5] = hepa.cnt;
    B_EEPROMTEMP[6] = (hepa.cnt >> 8) & 0XFF;
    B_EEPROMTEMP[7] = (hepa.cnt >> 16) & 0XFF;
    B_EEPROMTEMP[8] = (hepa.cnt >> 24) & 0XFF;
    B_EEPROMTEMP[9] = hepa.all_secs & 0XFF;
    B_EEPROMTEMP[10] = (hepa.all_secs >> 8) & 0XFF;
    B_EEPROMTEMP[11] = (hepa.all_secs >> 16) & 0XFF;
    B_EEPROMTEMP[12] = (hepa.all_secs >> 24) & 0XFF;
    B_EEPROMTEMP[13] = hepa.human;
    B_EEPROMTEMP[14] = 0;
		
		
    for(i = 0,sumcal = 0; i < 15; i++)
    {
        sumcal += B_EEPROMTEMP[i];
    }
    B_EEPROMTEMP[15] = sumcal;
    drive_hepa_save_data(type, B_EEPROMTEMP, 16);

    #ifdef DEBUG_PRINT_LEVEL
    DEBUG_PRINT_I("hepa type:%d write.\r\n", (int)type);
    #endif
}

static unsigned char __core_hepa_cal(TEKFlash_e type, TEKHepa_t *hepa, unsigned char *value)
{
    unsigned char  i;
    unsigned char sumcal = 0;
    for(i = 0; i < 15; i++)
    {
        sumcal += value[i];
    }
    if((value[15] == sumcal) && (value[0] == HEPA_WRITE_FLAG))
    {
        hepa->secs = (unsigned long)value[1] | ((unsigned long)value[2] << 8) | ((unsigned long)value[3] << 16) | ((unsigned long)value[4] << 24);
        hepa->start_secs = hepa->secs;
        hepa->cnt = (unsigned long)value[5] | ((unsigned long)value[6] << 8) | ((unsigned long)value[7] << 16) | ((unsigned long)value[8] << 24);
        hepa->all_secs = (unsigned long)value[9] | ((unsigned long)value[10] << 8) | ((unsigned long)value[11] << 16) | ((unsigned long)value[12] << 24);
        hepa->start_all_secs = hepa->all_secs;
        hepa->human =  value[13];
        #ifdef DEBUG_PRINT_LEVEL
			  DEBUG_PRINT_I("hepa type:%d,%uH:%uM:%uS,cnt:%u,all_sec:%uH:%uM:%uS,", \
			 (int)type, (unsigned int)(hepa->secs / 3600), (unsigned int)((hepa->secs % 3600) / 60), (unsigned int)(hepa->secs % 60), \
				hepa->cnt, (unsigned int)(hepa->all_secs / 3600), (unsigned int)((hepa->all_secs % 3600) / 60), (unsigned int)(hepa->all_secs % 60));
				 
				DEBUG_PRINT_I("human:%d\r\n",(int)(!!hepa->human));
        #endif
        return 1;
    }
    else
    {
        #ifdef DEBUG_PRINT_LEVEL
        DEBUG_PRINT_E("Read hepa %d checksum error!\r\n", (int)type);
        #endif			
			
        core_flash_write(type);

    }
    return 0;
}

static void __core_refresh_tick(void)
{
    tek_hepa.start_tick = Get_Sys_Tick();
    tek_hepa.start_secs = tek_hepa.secs;
    tek_hepa.start_all_secs = tek_hepa.all_secs;
    #ifdef HEPA2_TIMEOUT
    tek_hepa2.start_tick = Get_Sys_Tick();
    tek_hepa2.start_secs = tek_hepa2.secs;
    tek_hepa2.start_all_secs = tek_hepa2.all_secs;
    #endif
    #ifdef HEPA3_TIMEOUT
    tek_hepa3.start_tick = Get_Sys_Tick();
    tek_hepa3.start_secs = tek_hepa3.secs;
    tek_hepa3.start_all_secs = tek_hepa3.all_secs;
    #endif
}


//����HEPA����ʱ��
void core_hepa_add(TEKFlash_e type, TEKHepa_t *hepa)
{
    static unsigned int  step[5] = {0, 5, 10};
    unsigned long run_seconds = (Get_Sys_Tick() - hepa->start_tick) / 1000;
    if(core_get_tek_power()->on && (Get_Sys_Tick() - hepa->sec_tick >= 1000))
    {
			hepa->sec_tick = Get_Sys_Tick();
			if(!hepa->time_out)
			{
				hepa->secs = hepa->start_secs + run_seconds;
				hepa->all_secs = hepa->start_all_secs + run_seconds;
				step[type] = step[type] + 1;
				#if DEBUG_PRINT_ENABLE
				if(step[type] % 60 == 0)
				{
					printf("hepa%d:%d:%d:%d", (int)type, (unsigned long)(hepa->secs / 3600), (unsigned long)((hepa->secs % 3600) / 60), (unsigned long)(hepa->secs % 60));
				}
				#endif
				if(step[type] >= 3600)
				{
					step[type] = 0;
					core_flash_write(type);
					__core_refresh_tick();
				}
				
				hepa->left_days = (hepa->all - hepa->secs)/86400;						
				hepa->left_percent = 100 - hepa->secs/hepa->all*100;//(hepa->all - hepa->secs)*100ul/hepa->all;	
                if((hepa->all - hepa->secs)%hepa->all*100ul)	
                    hepa->left_percent += 1;		
			}
			else
			{
				hepa->left_percent = 0;
				hepa->left_days = 0;
				
			}		
    }
}



void core_hepa_handle(void)
{
    unsigned char  B_EEPROMTEMP[16] = {0};
    static unsigned char   power_last;
    static unsigned long  power_tick;
    static unsigned char  hepa_readed = 0;
    static unsigned char  b_register;
    #ifdef HEPA2_TIMEOUT
    static unsigned char  hepa_readed2 = 0;
    #endif
    #ifdef HEPA3_TIMEOUT
    static unsigned char  hepa_readed3 = 0;
    #endif
    if(!b_register)
    {
        b_register = 1;
        #if USE_TE
        core_tek_hepa_te_register(&tek_hepa);//����TE
        #ifdef HEPA2_TIMEOUT
        core_tek_hepa2_te_register(&tek_hepa2);//����TE
        #endif
        #ifdef HEPA3_TIMEOUT
        core_tek_hepa3_te_register(&tek_hepa3);//����TE
        #endif
        #endif //USE_TE
    
		#ifdef HEPA_TIMEOUT
		tek_hepa.all = HEPA_TIMEOUT;
		#endif
		#ifdef HEPA2_TIMEOUT
		tek_hepa2.all = HEPA2_TIMEOUT;
		#endif
		#ifdef HEPA3_TIMEOUT
		tek_hepa3.all = HEPA3_TIMEOUT;
		#endif		
	
	
	}
    if((core_get_tek_power()->on != power_last) && !core_get_tek_power()->on)
    {
        if(Get_Sys_Tick() - power_tick >= 30000)
        {
            power_tick = Get_Sys_Tick();
            __core_heap_write();
        }
    }
    power_last = core_get_tek_power()->on;
    if(tek_hepa.secs > HEPA_TIMEOUT)
    {
        if(!tek_hepa.time_out)
        {
            tek_hepa.time_out = 1;
            core_flash_write(TEK_ENUM_HEPA);
        }
    }
    else
    {
        tek_hepa.time_out = 0;
    }
    #ifdef HEPA2_TIMEOUT
    if(tek_hepa2.secs > HEPA2_TIMEOUT)
    {
        if(!tek_hepa2.time_out)
        {
            tek_hepa2.time_out = 1;
            core_flash_write(TEK_ENUM_HEPA2);
        }
    }
    else
    {
        tek_hepa2.time_out = 0;
    }
    #endif
    #ifdef HEPA3_TIMEOUT
    if(tek_hepa3.secs > HEPA3_TIMEOUT)
    {
        if(!tek_hepa3.time_out)
        {
            tek_hepa3.time_out = 1;
            core_flash_write(TEK_ENUM_HEPA3);
        }
    }
    else
    {
        tek_hepa3.time_out = 0;
    }
    #endif
    if(!core_get_tek_power()->on)
    {
        __core_refresh_tick();
		
	  #ifdef HEPA2_TIMEOUT
	  tek_hepa2.flash_time = 0;
	  #endif
	
	  #ifdef HEPA3_TIMEOUT
	  tek_hepa3.flash_time = 0;
	  #endif			
    }
    if(tek_hepa.reset)
    {
        tek_hepa.reset = 0;
        tek_hepa.secs = 0;
        tek_hepa.human = 0;
        core_flash_write(TEK_ENUM_HEPA);
        tek_hepa.start_tick = Get_Sys_Tick();
        tek_hepa.start_secs = tek_hepa.secs;
        tek_hepa.start_all_secs = tek_hepa.all_secs;
    }
		
    core_hepa_add(TEK_ENUM_HEPA, &tek_hepa);
    #ifdef HEPA2_TIMEOUT
    if(tek_hepa2.reset)
    {
        tek_hepa2.reset = 0;
        tek_hepa2.secs = 0;
        tek_hepa2.human = 0;
        core_flash_write(TEK_ENUM_HEPA2);
        tek_hepa2.start_tick = Get_Sys_Tick();
        tek_hepa2.start_secs = tek_hepa2.secs;
        tek_hepa2.start_all_secs = tek_hepa2.all_secs;
    }
    core_hepa_add(TEK_ENUM_HEPA2, &tek_hepa2);
    #endif
    #ifdef HEPA3_TIMEOUT
    if(tek_hepa3.reset)
    {
        tek_hepa3.reset = 0;
        tek_hepa3.secs = 0;
        tek_hepa3.human = 0;
        core_flash_write(TEK_ENUM_HEPA3);
        tek_hepa3.start_tick = Get_Sys_Tick();
        tek_hepa3.start_secs = tek_hepa3.secs;
        tek_hepa3.start_all_secs = tek_hepa3.all_secs;
    }
    core_hepa_add(TEK_ENUM_HEPA3, &tek_hepa3);
    #endif
    if(tek_flash_write_t.hepa)
    {
        tek_flash_write_t.hepa = 0;
        tek_hepa.cnt++;
        __core_hepa_data(TEK_ENUM_HEPA, tek_hepa);
    }
    #ifdef HEPA2_TIMEOUT
    if(tek_flash_write_t.hepa2)
    {
        tek_flash_write_t.hepa2 = 0;
        tek_hepa2.cnt++;
        __core_hepa_data(TEK_ENUM_HEPA2, tek_hepa2);
    }
    #endif
    #ifdef HEPA3_TIMEOUT
    if(tek_flash_write_t.hepa3)
    {
        tek_flash_write_t.hepa3 = 0;
        tek_hepa3.cnt++;
        __core_hepa_data(TEK_ENUM_HEPA3, tek_hepa3);
    }
    #endif
    if((hepa_readed == 0) && (Get_Sys_Tick() > 200))
    {
        hepa_readed = 1;
        drive_hepa_read_data(TEK_ENUM_HEPA, B_EEPROMTEMP, 16);
        #if USE_BACKUP_HEPA
        if(__core_hepa_cal(TEK_ENUM_HEPA, &tek_hepa, B_EEPROMTEMP) == 0)
        {
            drive_hepa_read_backup_data(TEK_ENUM_HEPA, B_EEPROMTEMP, 16);
            __core_hepa_cal(TEK_ENUM_HEPA, &tek_hepa, B_EEPROMTEMP);
        }
        #else
        __core_hepa_cal(TEK_ENUM_HEPA, &tek_hepa, B_EEPROMTEMP);
        #endif
    }
    #ifdef HEPA2_TIMEOUT
    if((hepa_readed2 == 0) && (Get_Sys_Tick() > 200))
    {
        hepa_readed2 = 1;
        drive_hepa_read_data(TEK_ENUM_HEPA2, B_EEPROMTEMP, 16);
        #if USE_BACKUP_HEPA
        if(__core_hepa_cal(TEK_ENUM_HEPA2, &tek_hepa2, B_EEPROMTEMP) == 0)
        {
            drive_hepa_read_backup_data(TEK_ENUM_HEPA2, B_EEPROMTEMP, 16);
            __core_hepa_cal(TEK_ENUM_HEPA2, &tek_hepa2, B_EEPROMTEMP);
        }
        #else
        __core_hepa_cal(TEK_ENUM_HEPA2, &tek_hepa2, B_EEPROMTEMP);
        #endif
    }
    #endif
    #ifdef HEPA3_TIMEOUT
    if((hepa_readed3 == 0) && (Get_Sys_Tick() > 200))
    {
        hepa_readed3 = 1;
        drive_hepa_read_data(TEK_ENUM_HEPA3, B_EEPROMTEMP, 16);
        #if USE_BACKUP_HEPA
        if(__core_hepa_cal(TEK_ENUM_HEPA3, &tek_hepa3, B_EEPROMTEMP) == 0)
        {
            drive_hepa_read_backup_data(TEK_ENUM_HEPA3, B_EEPROMTEMP, 16);
            __core_hepa_cal(TEK_ENUM_HEPA3, &tek_hepa3, B_EEPROMTEMP);
        }
        #else
        __core_hepa_cal(TEK_ENUM_HEPA3, &tek_hepa3, B_EEPROMTEMP);
        #endif
    }
    #endif
}

#endif  //HEPA_TIMEOUT
