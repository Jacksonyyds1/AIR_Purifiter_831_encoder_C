
#include "ihastek.h"
#include <stdio.h>

#define SAVEDATACOUNT  80//hepa1 16;hepa2 16; te 8;userdata+product 40(1header+1checksum+2userdata+32productdata)
#define USERDATAOFFSET    40
static unsigned char  save_data[SAVEDATACOUNT];
static unsigned char  is_save;
static unsigned long  save_tick;

void drive_dev_read_data(unsigned char *value, unsigned char len);

#ifdef HEPA_TIMEOUT
void drive_hepa_save_data(TEKFlash_e type, unsigned char *value, unsigned char len)
{
    is_save = 1;
    save_tick = Get_System_Tick();
    switch(type)
    {
    case TEK_ENUM_HEPA:
    {
        memcpy(save_data, value, len);
    } break;
#ifdef HEPA2_TIMEOUT
    case TEK_ENUM_HEPA2:
    {
        memcpy(save_data + 16, value, len);
    } break;
#endif
#ifdef HEPA3_TIMEOUT
    case TEK_ENUM_HEPA3:
    {
        memcpy(save_data + 48, value, len);
    } break;
#endif
    default:
        break;
    }
}
#endif  //HEPA_TIMEOUT

#ifdef HEPA_TIMEOUT
void drive_hepa_read_data(TEKFlash_e type, unsigned char *value, unsigned char len)
{

    drive_dev_read_data(save_data, SAVEDATACOUNT);
    switch(type)
    {
    case TEK_ENUM_HEPA:
    {
        memcpy(value, save_data, len);

    } break;
#ifdef HEPA2_TIMEOUT
    case TEK_ENUM_HEPA2:
    {
        memcpy(value, save_data + 16, len);
    } break;
#endif
#ifdef HEPA3_TIMEOUT
    case TEK_ENUM_HEPA3:
    {
        memcpy(value, save_data + 48, len);
    } break;
#endif
    default:
        break;
    }
}
#endif

#if USE_BACKUP_HEPA
void drive_dev_read_backup_data(unsigned char *value, unsigned char len);

static unsigned char  save_backup_data[64];//16*3+8+8=64
void drive_hepa_read_backup_data(TEKFlash_e type, unsigned char *value, unsigned char len)
{
    drive_dev_read_data(save_backup_data, 64);

    switch(type)
    {
    case TEK_ENUM_HEPA:
    {
        memcpy(value, save_backup_data, len);
    } break;
    case TEK_ENUM_HEPA2:
    {
        memcpy(value, save_backup_data + 32, len);
    } break;
    case TEK_ENUM_HEPA3:
    {
        memcpy(value, save_backup_data + 48, len);
    } break;
    default:
        break;
    }

}
#endif

#if USE_TE

void drive_dev_addr_save_data(unsigned char *value, unsigned char len)
{

    is_save = 1;
    save_tick = Get_System_Tick();

    memcpy(save_data + 32, value, len);

}

void drive_dev_addr_read_data(unsigned char *value, unsigned char len)
{

    drive_dev_read_data(save_data, SAVEDATACOUNT);

    memcpy(value, save_data + 32, len);

}
#endif   //USE_TE

#if USE_STATUS_WRITE //�����豸����״̬

void drive_status_save_data(unsigned char *value, unsigned char len)
{
    is_save = 1;
    save_tick = Get_System_Tick();
    memcpy(save_data + USERDATAOFFSET, value, len);
}
void drive_status_read_data(unsigned char *value, unsigned char len)
{
    drive_dev_read_data(save_data, SAVEDATACOUNT);

    memcpy(value, save_data + USERDATAOFFSET, len);
}

#endif

void drive_dev_save_data(unsigned char *value, unsigned int len);//����HEPA,VOC,PCO ��о���ݼ��豸��ַ��״̬д�����

#if USE_BACKUP_HEPA
void drive_dev_save_backup_data(unsigned char *value, unsigned char len);//����HEPA,VOC,PCO ��о���ݼ��豸��ַ��״̬д����� �ı���

#endif

unsigned char does_flash_write(void)//���� 0������д 1������д
{

    static unsigned long  detect_tick5;
    static unsigned long  detect_tick3600;
    static unsigned long  write_times5;
    static unsigned long  write_times3600;
    static unsigned char  No_write;

    //5����д�Ĵ�������5��д�Ĵ�������10�Σ���ر�д�Ĺ���
    write_times5++;
    if(Get_Sys_Tick() - detect_tick5 > 5000)
    {
        detect_tick5 = Get_Sys_Tick();
        if(write_times5 > 10)
        {
            No_write = 1;

#ifdef DEBUG_PRINT_LEVEL
            DEBUG_PRINT_E("Flash write fast ERROR!!!\r\n");
#endif

        }
        write_times5 = 0;

    }
    //3600����д�Ĵ�������3600��д�Ĵ�������50�Σ���ر�д�Ĺ���
    write_times3600 = (write_times3600) + 1;
    if(Get_Sys_Tick() - detect_tick3600 > 3600000)
    {
        detect_tick3600 = Get_Sys_Tick();
        if(write_times3600 > 60)
        {
            No_write = 1;
#ifdef DEBUG_PRINT_LEVEL
            DEBUG_PRINT_E("Flash write fast ERROR!!!\r\n");
#endif

        }
        write_times3600 = 0;
    }
    if(No_write)
    {
        //�ر�д�Ĺ��ܺ�ÿСʱ��ͨдһ��
        static unsigned long  No_write_tick;
        if(Get_Sys_Tick() - No_write_tick >= 3600000)
        {
            No_write_tick = Get_Sys_Tick();
            if(core_get_tek_power()->on)
            {
                return 1;
            }
        }

        return 0;
    }

    return 1;

}

void core_drive_save(void)
{
    //  unsigned char dataarr[64];
    if(is_save && ((Get_System_Tick() - save_tick >= 60) || (core_get_tek_power()->needSaveImmediately)))
    {
        is_save = 0;

        if(!core_get_tek_power()->needSaveImmediately)
        {
            if(does_flash_write() == 0)
            {
                return;    //�ж��ǲ���д��̫Ƶ����̫Ƶ���˳�����
            }
        }
        drive_dev_save_data(save_data, SAVEDATACOUNT);
#if USE_BACKUP_HEPA
        drive_dev_save_backup_data(save_data, 64);
#endif
        core_get_tek_power()->needSaveImmediately = 0;
    }
}
