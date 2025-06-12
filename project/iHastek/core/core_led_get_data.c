#include "ihastek.h"


#if USE_LED_GET_VERSION_HEPA

static tek_u8 XDATA_T B_DataCnt2, B_DataCounter2, B_DataCounter_12;
static tek_u8 XDATA_T Tmp_D1[6], Tmp_D2[6], Tmp_index;

void drive_get_data_led_on(tek_u8 *dis);
void drive_get_data_led_off(tek_u8 *dis);


static tek_u8 _core_get_version_index(char *ver)
{
    int i = 0;
    static unsigned char b_get_index;
    static unsigned char get_index;
    if(b_get_index == 0)
    {
        b_get_index = 1;
        for(i = 0;; i++)
        {
            if(ver[i] == '_' || ver[i] == '-')
            { break; }
        }
        get_index = i;
        get_index += 2;
    }
    return   get_index;
}

static void _core_get_version_value(char *ver, tek_u8 *dat)
{
    unsigned char index;
    index = _core_get_version_index(ver);
    dat[0] = ver[index + 3] - 0x30;
    dat[0] |= (ver[index + 2] - 0x30) << 3;
    dat[0] |= (ver[index] - 0X30) << 6;
    dat[1] = (ver[index + 3] - 0x30) + (ver[index + 2] - 0x30) + (ver[index] - 0X30);
}

static void _core_get_hepa_value(tek_u32 hepa, tek_u8 *dat)
{
    dat[0] = hepa;
    dat[1] = hepa >> 8;
    dat[2] = hepa >> 16;
    dat[3] = hepa >> 24;
    dat[4] = dat[0] + dat[1] + dat[2] + dat[3];
}


static void _core_te_get_dev_data(tek_u8 *dis, tek_u8 *value, tek_u8 bit_cnt)
{
    int i = 0;
    B_DataCounter2++;
    if(B_DataCounter2 < 4)
    {
        drive_get_data_led_off(dis);
    }
    else
    {
        drive_get_data_led_on(dis);
        if(B_DataCnt2 == 0)
        {
            if(B_DataCounter2 >= 64)
            {
                drive_get_data_led_off(dis);
                B_DataCounter2 = 0;
                B_DataCnt2++;
                Tmp_index = 0;
                for(i = 0; i < 5; i++)
                {
                    if(i >= (bit_cnt / 8 + 1))
                    { break; }
                    Tmp_D1[i] = value[i];
                }
            }
        }
        else if(B_DataCnt2 <= bit_cnt)
        {
            if(B_DataCounter2 == 4)
            {
                B_DataCounter_12 = 4 + 4;
                if(Tmp_D1[Tmp_index] & 0x01)
                { B_DataCounter_12 = 4 + 12; }
                Tmp_D1[Tmp_index] >>= 1;
            }
            if(B_DataCounter_12 == B_DataCounter2)
            {
                if(B_DataCnt2 % 8 == 0)
                {
                    Tmp_index++;
                }
                drive_get_data_led_off(dis);
                B_DataCounter2 = 0;
                B_DataCnt2++;
            }
        }
        else
        {
            if(B_DataCounter2 >= 28 + 4)
            {
                drive_get_data_led_off(dis);
                B_DataCounter2 = 0;
                B_DataCnt2 = 0;
            }
        }
    }
}


void core_led_get_data(tek_u8 *led)//参数为要控制的数组，如tm1668
{
    switch(core_get_tek_power()->led_get_data)
    {
        case 1:
        {
            _core_get_version_value(DEV_VERSION, Tmp_D2);
            _core_te_get_dev_data(led, Tmp_D2, 10);
        }
        break;
        #ifdef HEPA_TIMEOUT
        case 2:
        {
            _core_get_hepa_value(core_get_hepa()->secs, Tmp_D2);
            _core_te_get_dev_data(led, Tmp_D2, 40);
        }
        break;
        #endif
        #ifdef HEPA2_TIMEOUT
        case 3:
        {
            _core_get_hepa_value(core_get_hepa2()->secs, Tmp_D2);
            _core_te_get_dev_data(led, Tmp_D2, 40);
        }
        break;
        #endif
        #ifdef HEPA3_TIMEOUT
        case 2:
        {
            _core_get_hepa_value(core_get_hepa3()->secs, Tmp_D2);
            _core_te_get_dev_data(led, Tmp_D2, 40);
        }
        break;
        #endif
        default:
            break;
    }
}



#endif


#if   USE_NIXIE_TUBE_GET_VERSION_HEPA

static void _core_get_version_to_digit(char *ver, tek_u8 *digit)
{
    int i = 0;
    unsigned char get_index = 0;
    for(i = 0;; i++)
    {
        if(ver[i] == '_' || ver[i] == '-')
        { break; }
    }
    get_index = i;
    get_index += 2;
    digit[0] = ver[get_index] - 0x30;
    digit[1] = ver[get_index + 2] - 0x30;
    digit[2] = ver[get_index + 3] - 0x30;
}

static void _core_get_hepa_value(tek_u32 hepa, tek_u8 *dat)
{
    tek_u32 hepa_hour = hepa / 3600;
    dat[0] = hepa_hour / 1000;
    dat[1] = hepa_hour % 1000 / 100;
    dat[2] = hepa_hour % 100 / 10;
    dat[3] = hepa_hour % 10;
}


void core_led_get_data(tek_u8 *led)
{
    switch(core_get_tek_power()->led_get_data)
    {
        case 1:
        {
            _core_get_version_to_digit(DEV_VERSION, led);
        }
        break;
        #ifdef HEPA_TIMEOUT
        case 2:
        {
            _core_get_hepa_value(core_get_hepa()->secs, led);
        }
        break;
        #endif
        #ifdef HEPA2_TIMEOUT
        case 3:
        {
            _core_get_hepa_value(core_get_hepa2()->secs, led);
        }
        break;
        #endif
        #ifdef HEPA3_TIMEOUT
        case 3:
        {
            _core_get_hepa_value(core_get_hepa3()->secs, led);
        }
        break;
        #endif
        default:
            break;
    }
}

#endif

