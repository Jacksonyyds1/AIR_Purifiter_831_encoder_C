#include "ihastek.h"
#include "user_config.h"
#include "display.h"
#if USE_POWER

static TEKPower_t tek_power;
TEKPower_t *core_get_tek_power(void)
{
    return &tek_power;
}


void core_tek_power_register(void)
{
    #if USE_TE
    core_tek_power_te_register(&tek_power);
    #endif
}



static void _core_cal_power_on(void)
{
    static unsigned long  power_off_tick;
    if(tek_power.on)
    {
        tek_power.on_secs = (Get_Sys_Tick() - power_off_tick) / 1000;
    }
    else
    {
        power_off_tick = Get_Sys_Tick();
//        tek_power.on_secs = 0;
 //       platform_power_off();
    }
}

#if USE_STATUS_WRITE 
void UserFactoryReset(void)
{
	core_get_tek_power()->hadDisplayLOGO = 0;
	core_hepa_reset();core_hepa2_reset();
	tek_power.usersetwifiStatus = 1;
	tek_power.PM25SENSORMORING = 0;
	tek_power.speed.changetype = SPEEDCHANGETYPE_USER;
	tek_power.speed.speedpara[tek_power.speed.changetype].mode = SPEEDMODE_AUTO;
	tek_power.speed.speedpara[tek_power.speed.changetype].lev = SPEEDLEVEL_1;
	core_get_tek_power()->needSaveImmediately = 1;
	core_get_tek_power()->aconFlagForSendWIFIEnable = 0;
	core_get_tek_power()->sensordata.count = 0;
	core_get_tek_power()->facModeTick = Get_System_Tick();
}

static unsigned long  status_write_cnt;
void drive_status_save_data(unsigned char *value,unsigned char len);
#define  STATUS_FLAG  0X5B
static unsigned char  status_backup[2+32];
static void _core_status_write(void)
{
	static unsigned long  save_tick;
	unsigned char  /*statusFlag=0,*/status_new[8+32]={0};
	unsigned char i=0,counter = 0;
	static unsigned char  b_save=0;
	unsigned char check=0;
	
	
	status_new[0] = STATUS_FLAG;
	if(tek_power.usersetwifiStatus)
		status_new[1] = 0x01;
	if(tek_power.PM25SENSORMORING)
		status_new[1] |= 0x02;
	if(tek_power.hadDisplayLOGO)
		status_new[1] |= 0x04;
	status_new[2] = 	tek_power.speed.speedpara[tek_power.speed.changetype].mode;
	status_new[2] <<= 4;
	status_new[2] |= 	tek_power.speed.speedpara[tek_power.speed.changetype].lev;
	i = 3;
	status_new[i++] = core_get_tek_power()->productData.secsofAQPage & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofAQPage>>8) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofAQPage>>16) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofAQPage>>24) & 0xff;
	status_new[i++] = core_get_tek_power()->productData.secsofPM25Page & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofPM25Page>>8) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofPM25Page>>16) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofPM25Page>>24) & 0xff;
	status_new[i++] = core_get_tek_power()->productData.secsofPM10Page & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofPM10Page>>8) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofPM10Page>>16) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofPM10Page>>24) & 0xff;
	status_new[i++] = core_get_tek_power()->productData.secsofFilterLifePage & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofFilterLifePage>>8) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofFilterLifePage>>16) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofFilterLifePage>>24) & 0xff;
	status_new[i++] = core_get_tek_power()->productData.secsofWIFIStatusPage & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofWIFIStatusPage>>8) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofWIFIStatusPage>>16) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.secsofWIFIStatusPage>>24) & 0xff;
	status_new[i++] = core_get_tek_power()->productData.numofPowerButton & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.numofPowerButton>>8) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.numofPowerButton>>16) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.numofPowerButton>>24) & 0xff;
	status_new[i++] = core_get_tek_power()->productData.numofModeButton & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.numofModeButton>>8) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.numofModeButton>>16) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.numofModeButton>>24) & 0xff;
	status_new[i++] = core_get_tek_power()->productData.numofInfoButton & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.numofInfoButton>>8) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.numofInfoButton>>16) & 0xff;
	status_new[i++] = (core_get_tek_power()->productData.numofInfoButton>>24) & 0xff;
	counter = i;
	for(i=0;i<counter;i++)
	{
		if(status_backup[i]!=status_new[i+1])
		{
			b_save=1;
			save_tick = Get_System_Tick();
		}
		status_backup[i]=status_new[i+1];
	}
	
	if(core_get_tek_power()->needSaveImmediately || (b_save && (Get_System_Tick() - save_tick>1)))//״̬�ı䣬��ʱ1�뱣�棬������Ϊ�Ұ�
	{
		b_save=0;
		status_write_cnt++;	
		
		status_new[counter++] = status_write_cnt & 0xff;
		status_new[counter++] = (status_write_cnt>>8) & 0xff;
		status_new[counter++] = (status_write_cnt>>16) & 0xff;
		status_new[counter++] = (status_write_cnt>>24) & 0xff;

		for(i=0,check=0;i<counter;i++)
		{
			check +=status_new[i];
			
		}		
		status_new[counter++] = check;
		drive_status_save_data(status_new,counter);
	}
}

void drive_status_read_data(unsigned char *value, unsigned char len);
static void _core_status_read(void)
{
	static unsigned char readFlag = 0;
	if(Get_Sys_Tick()>200 && !readFlag)
	{
		unsigned char  status_new[40]={0};
		unsigned char i=0;
		unsigned char check=0,counter = 40;

		readFlag = 1;
		drive_status_read_data(status_new,counter);
		for(i=0,check=0;i<counter-1;i++)
		{
			check +=status_new[i];
		}
		if((check==status_new[counter-1]) && (status_new[0]==STATUS_FLAG))
		{		
			if(status_new[1] & 0x01)
				tek_power.usersetwifiStatus = 1;
			else
				tek_power.usersetwifiStatus = 0;
			if(status_new[1] & 0x02)
				tek_power.PM25SENSORMORING = 1;
			else
				tek_power.PM25SENSORMORING = 0;
			if(status_new[1] & 0x04)
				tek_power.hadDisplayLOGO = 1;
			else
				tek_power.hadDisplayLOGO = 0;
			tek_power.speed.speedpara[tek_power.speed.changetype].mode = (SPEEDMODE)(status_new[2]>>4);
			tek_power.speed.speedpara[tek_power.speed.changetype].lev = (SPEEDLEVEL)(status_new[2]&0x0f);
			i = 3;
			core_get_tek_power()->productData.secsofAQPage = status_new[i++];
			core_get_tek_power()->productData.secsofAQPage |= ((unsigned long)status_new[i++]<<8) & 0xff;
			core_get_tek_power()->productData.secsofAQPage |= ((unsigned long)status_new[i++]<<16) & 0xff;
			core_get_tek_power()->productData.secsofAQPage |= ((unsigned long)status_new[i++]<<24) & 0xff;

			core_get_tek_power()->productData.secsofPM25Page = status_new[i++];
			core_get_tek_power()->productData.secsofPM25Page |= ((unsigned long)status_new[i++]<<8) & 0xff;
			core_get_tek_power()->productData.secsofPM25Page |= ((unsigned long)status_new[i++]<<16) & 0xff;
			core_get_tek_power()->productData.secsofPM25Page |= ((unsigned long)status_new[i++]<<24) & 0xff;

			core_get_tek_power()->productData.secsofPM10Page = status_new[i++];
			core_get_tek_power()->productData.secsofPM10Page |= ((unsigned long)status_new[i++]<<8) & 0xff;
			core_get_tek_power()->productData.secsofPM10Page |= ((unsigned long)status_new[i++]<<16) & 0xff;
			core_get_tek_power()->productData.secsofPM10Page |= ((unsigned long)status_new[i++]<<24) & 0xff;

			core_get_tek_power()->productData.secsofFilterLifePage = status_new[i++];
			core_get_tek_power()->productData.secsofFilterLifePage |= ((unsigned long)status_new[i++]<<8) & 0xff;
			core_get_tek_power()->productData.secsofFilterLifePage |= ((unsigned long)status_new[i++]<<16) & 0xff;
			core_get_tek_power()->productData.secsofFilterLifePage |= ((unsigned long)status_new[i++]<<24) & 0xff;

			core_get_tek_power()->productData.secsofWIFIStatusPage = status_new[i++];
			core_get_tek_power()->productData.secsofWIFIStatusPage |= ((unsigned long)status_new[i++]<<8) & 0xff;
			core_get_tek_power()->productData.secsofWIFIStatusPage |= ((unsigned long)status_new[i++]<<16) & 0xff;
			core_get_tek_power()->productData.secsofWIFIStatusPage |= ((unsigned long)status_new[i++]<<24) & 0xff;

			core_get_tek_power()->productData.numofPowerButton = status_new[i++];
			core_get_tek_power()->productData.numofPowerButton |= ((unsigned long)status_new[i++]<<8) & 0xff;
			core_get_tek_power()->productData.numofPowerButton |= ((unsigned long)status_new[i++]<<16) & 0xff;
			core_get_tek_power()->productData.numofPowerButton |= ((unsigned long)status_new[i++]<<24) & 0xff;

			core_get_tek_power()->productData.numofModeButton = status_new[i++];
			core_get_tek_power()->productData.numofModeButton |= ((unsigned long)status_new[i++]<<8) & 0xff;
			core_get_tek_power()->productData.numofModeButton |= ((unsigned long)status_new[i++]<<16) & 0xff;
			core_get_tek_power()->productData.numofModeButton |= ((unsigned long)status_new[i++]<<24) & 0xff;

			core_get_tek_power()->productData.numofInfoButton = status_new[i++];
			core_get_tek_power()->productData.numofInfoButton |= ((unsigned long)status_new[i++]<<8) & 0xff;
			core_get_tek_power()->productData.numofInfoButton |= ((unsigned long)status_new[i++]<<16) & 0xff;
			core_get_tek_power()->productData.numofInfoButton |= ((unsigned long)status_new[i++]<<24) & 0xff;

		    status_write_cnt = (status_new[i++]) & 0xff;
		    status_write_cnt |= ((unsigned long)status_new[i++]<<8) & 0xff;
			status_write_cnt |= ((unsigned long)status_new[i++]<<16) & 0xff;
			status_write_cnt |= ((unsigned long)status_new[i++]<<24) & 0xff;
			
			counter = i;
			for(i=0;i<counter;i++)
			{
				status_backup[i]=status_new[i+1];
			}
			// status_backup[0] = status_new[1];  //
			// status_backup[1] = status_new[2];		
		}
		else
		{
			tek_power.hadDisplayLOGO = 0;
			tek_power.usersetwifiStatus = 1;
			tek_power.PM25SENSORMORING = 0;
			tek_power.speed.changetype = SPEEDCHANGETYPE_USER;
			tek_power.speed.speedpara[tek_power.speed.changetype].mode = SPEEDMODE_AUTO;
			tek_power.speed.speedpara[tek_power.speed.changetype].lev = SPEEDLEVEL_1;
			core_get_tek_power()->productData.secsofAQPage = 0;
			core_get_tek_power()->productData.secsofPM25Page = 0;
			core_get_tek_power()->productData.secsofPM10Page = 0;
			core_get_tek_power()->productData.secsofFilterLifePage = 0;
			core_get_tek_power()->productData.secsofWIFIStatusPage = 0;
			core_get_tek_power()->productData.numofPowerButton = 0;
			core_get_tek_power()->productData.numofModeButton = 0;
			core_get_tek_power()->productData.numofInfoButton = 0;
			status_write_cnt = 0;
		}		
	}
	
}

unsigned long core_get_status_write_cnt(void)
{
	return status_write_cnt;
	
}

#endif

void core_power_status(void)
{
	static unsigned char  productdata_read = 0;
	unsigned char readDataarrayTemp[24*3*20];

	_core_cal_power_on();
	#if USE_STATUS_WRITE  
	_core_status_write();
	_core_status_read();
	#endif
	if(!productdata_read)
	{
		productdata_read = 1;
		
// 		drive_EnvData_read_data(readDataarrayTemp,1);
// 		core_get_tek_power()->sensordataForAPP.groupIndex = readDataarrayTemp[0];
// 		if((core_get_tek_power()->sensordataForAPP.groupIndex > 0) && (core_get_tek_power()->sensordataForAPP.groupIndex <= 24*3))
// 		{
// 			core_get_tek_power()->sensordataForAPP.groupIndex = readDataarrayTemp[0];
// //			drive_EnvData_read_data(core_get_tek_power()->sensordataForAPP.,core_get_tek_power()->sensordataForAPP.groupIndex * 20);
// 		}
	
	}
}

#endif
