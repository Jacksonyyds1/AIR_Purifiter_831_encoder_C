#include "ihastek.h"
#include "platform.h"
#include "at32f403a_407_gpio.h"
#include "lcd.h"
#include "scd40.h"
#include "sen68.h"
#include "opt3004.h"
#include "encoder.h"

#if USE_LED_TEST
TEKLed_t   TE_led;//TE_led.all_on����ȫ��������TE_led.num�������������Ƶ���,������ʵ�ʵ����Ƶ��߼�
#endif

#if  USE_PM25
#if  USE_PPD42
uint16_t   ppd_low;
#endif
#endif

#if USE_TIMER
//auto_off:Action to be performed when time is up
void platform_auto_off(void)
{
    core_get_tek_power()->on = 0;
}

#endif

#if USE_POWER
void platform_power_off(void)
{
    core_get_tek_power()->on = 0;

}

void platform_power_up_init(void)
{
    core_get_tek_power()->on = 0;
    core_get_tek_power()->speed.changetype = SPEEDCHANGETYPE_USER;
    core_get_tek_power()->speed.speedpara[0].lev = SPEEDLEVEL_1; //user
    core_get_tek_power()->speed.speedpara[0].mode = SPEEDMODE_MANUAL; //user
}
#endif

#if USE_FAN_PWM

const uint16_t speed_rpm[] =
{
    0,
    250,//1 LOW
    1500,
    1550,//8
    1600,
    1650,//5
    1700,//9
};
static TEKSpeed_t tek_speed;

#endif

#if USE_PM25
const TEK_PM25_AUTO_SPEED_t Code_PM25[] =
{
    {1, 50},//40,
    {5, 75},//70,
    {6, 100},//90,
    {7, 350},//110,
    {8, 420},//130,
};
#endif

#if USE_TM1668

#if 0  //�߶��룬����ʱΪ0

#define CB_SEGA             (SEG1)
#define CB_SEGB             (SEG2)
#define CB_SEGC             (SEG3)
#define CB_SEGD             (SEG4)
#define CB_SEGE             (SEG5)
#define CB_SEGF             (SEG6)
#define CB_SEGG             (SEG7)

CONST_T unsigned char   CODE_T Code_7SEG[] =
{
    CB_SEGA + CB_SEGB + CB_SEGC + CB_SEGD + CB_SEGE + CB_SEGF,//0
    CB_SEGB + CB_SEGC,  //1
    CB_SEGA + CB_SEGB + CB_SEGD + CB_SEGE + CB_SEGG,//2
    CB_SEGA + CB_SEGB + CB_SEGC + CB_SEGD + CB_SEGG,//3
    CB_SEGB + CB_SEGC + CB_SEGF + CB_SEGG,//4
    CB_SEGA + CB_SEGC + CB_SEGD + CB_SEGF + CB_SEGG,    //5
    CB_SEGA + CB_SEGC + CB_SEGD + CB_SEGE + CB_SEGF + CB_SEGG,//6
    CB_SEGA + CB_SEGB + CB_SEGC,//7
    CB_SEGA + CB_SEGB + CB_SEGC + CB_SEGD + CB_SEGE + CB_SEGF + CB_SEGG,//8
    CB_SEGA + CB_SEGB + CB_SEGC + CB_SEGD + CB_SEGF + CB_SEGG,//9
    CB_SEGA + CB_SEGB + CB_SEGC + CB_SEGE + CB_SEGF + CB_SEGG,//A
    CB_SEGC + CB_SEGD + CB_SEGE + CB_SEGF + CB_SEGG,//B
    CB_SEGA + CB_SEGD + CB_SEGE + CB_SEGF,//C
    //CB_SEGB + CB_SEGC + CB_SEGD + CB_SEGE + CB_SEGG,//D
    CB_SEGB + CB_SEGC  + CB_SEGE + CB_SEGF + CB_SEGG, //D
    0,//E
    //CB_SEGA + CB_SEGE + CB_SEGF + CB_SEGG,//F
    CB_SEGG,//F
    CB_SEGA + CB_SEGB + CB_SEGE + CB_SEGF + CB_SEGG,//P
    CB_SEGD + CB_SEGE + CB_SEGF,//L
    CB_SEGE + CB_SEGF,//I
};
#endif

// ʹ��TM1668_DISPLAY_SET(value,GRID,SEG)������ݣ�����ֱ��value[GRID1_SEG2] |=SEG2
void platform_tm1668_display(unsigned char *value)
{

    if(sys_tick_get() < 1000 || TE_led.all_on) //�ϵ�1����ȫ���Ƶ���
    {
        memset(value, 0, 14);
    }
    else if(TE_led.num)//TE����LED�ƣ�һ��һ�ŵ���
    {
        switch(TE_led.num)
        {
        case 1:
        {
            value[GRID3_SEG4] |= SEG4;
        } break;
        case 2:
        {
            value[GRID3_SEG5] |= SEG5;
        } break;
        case 3:
        {
            value[GRID3_SEG6] |= SEG6;
        } break;

        default: break;

        }
    }
    else if(core_get_tek_power()->on)
    {
        if(core_get_hepa()->flash_time)//��HEPA��3��
        {
            if((sys_tick_get() - core_get_hepa()->flash_tick > 500) && (sys_tick_get() - core_get_hepa()->flash_tick < 1000))
            {
                value[GRID3_SEG6] |= SEG6;
            }

            if(sys_tick_get() - core_get_hepa()->flash_tick >= 1000)
            {
                core_get_hepa()->flash_tick = sys_tick_get();
                core_get_hepa()->flash_time--;
            }

        }
        else if(core_get_hepa()->human && !core_get_hepa()->time_out)//hepa ���ʱ��˸
        {
            if((sys_tick_get() - core_get_hepa()->flash_tick > 500) && (sys_tick_get() - core_get_hepa()->flash_tick < 1000))
            {
                value[GRID3_SEG6] |= SEG6;
            }

            if(sys_tick_get() - core_get_hepa()->flash_tick >= 1000)
            {
                core_get_hepa()->flash_tick = sys_tick_get();
            }

        }
        else if(core_get_hepa()->time_out)//hepa������������
        {
            value[GRID3_SEG6] |= SEG6;
        }
    }
    else
    {

    }

}

unsigned char core_get_tm1668_brightness(void) //tm1668��������
{
    return Bin(10001111);
}

#if USE_TM1668_KEY
void platform_tm1668_key(unsigned char *value)
{
    value = value;
}
#endif //USE_TM1668_KEY

#endif  //USE_TM1668

#if  USE_KEY
unsigned char core_get_key_value(void)
{
    unsigned char keyValueTemp = 0;

    gpio_input_data_bit_read(KEY_POWER_PORT, KEY_POWER_PIN) ? (keyValueTemp &= ~KEY_POWER) : (keyValueTemp |= KEY_POWER);

    return keyValueTemp;
}

typedef enum
{
    EM_STEP_0 = 0,
    EM_STEP_START,
    EM_STEP_MAX
} EMSTEP;
EMSTEP engineeringModeStep = 0;

#if USE_KEY_RELEASE
void core_key_release(uint16_t key)
{
}
#endif

#if USE_KEY_TOUCH
void core_key_touch(uint16_t key)
{
}
#endif

#ifdef KEY_LONG_PRESS_TIME
void core_key_long_press(uint16_t key)
{
}
#endif

#if KEY_SECOND_LONG_PRESS_TIME//5s
void core_key_second_long_press(uint16_t key)
{
}
#endif

#if KEY_THIRDLY_LONG_PRESS//10s
void core_key_thirdly_long_press(uint16_t key)
{
}
#endif

#endif

#if USE_UART_PROTOCOL

void platform_uart_protocol(unsigned char cmd, unsigned char *value)
{
    switch(cmd)
    {
    case 0x50:
    {
        //key_value_tem = value[0];
    }
    break;
    default:
        break;
    }
}

#if USE_UART_SELF_PROTOCOL  //������Զ����Э���޸�
extern uint8_t usart2_tx_buffer[255];
void platform_uart_protocol_sent(uint16_t len, unsigned char cmd, unsigned char *payload)
{
    //    unsigned char uart_ptx[10];
    //    unsigned char i, checksum;
    //    uint16_t all_len = len + 4;
    //    checksum = 0;
    //    uart_ptx[0] = 0xc5;
    //    uart_ptx[1] = all_len & 0xff;
    //    uart_ptx[2] = (all_len >> 8) & 0xff;
    //    uart_ptx[3] = 0x01;//cmd;
    //    for(i = 0, checksum = 0; i < 4; i++)
    //    {
    //        checksum += uart_ptx[i];
    //    }
    //    for(i = 0; i < len; i++)
    //    {
    //        uart_ptx[i + 4] = payload[i];
    //        checksum += payload[i];
    //    }
    //    uart_ptx[i + 4] = checksum;
    memcpy(&usart2_tx_buffer[0], payload, 5);
    dma_DIPCLIB_writebytes(5);
    //    for(i = 0; i < all_len; i++)
    //    {
    //        drive_protocol_uart_send(uart_ptx[i]);
    //    }
}

void platform_uart_protocol_Handler(unsigned char *cmdState, unsigned char *b_rec, unsigned char Data) //���մ������Զ���Э�������޸�����
{
    static unsigned char     checksum, cmd;
    static unsigned char     palyload_index;
    static uint16_t    rev_len/*, rev_addr*/;
    static unsigned char     rev_payload[30];
    switch(*cmdState)
    {
    case 0:
        if(Data == 0xC5)
        {
            checksum = Data;
            *cmdState = 1;
        }
        break;
    case 1:
        rev_len = Data;
        *cmdState = 2;
        checksum += Data;
        break;
    case 2:
        rev_len |= (uint16_t)Data << 8;
        *cmdState = 3;
        checksum += Data;
        if(rev_len - 7 > 30)
        {
            cmdState = 0;
        }
        break;
    case 3:
        cmd = Data;
        checksum += Data;
        if(rev_len > 5)
        {
            *cmdState = 4;
        }
        else
        {
            *cmdState = 5;
        }
        palyload_index = 0;
        break;
    case 4:
        if(palyload_index >= 30)
        {
            palyload_index = 0;
            *cmdState = 0;
            break;
        }
        rev_payload[palyload_index++] = Data;
        checksum += Data;
        if(palyload_index >= (rev_len - 5))
        {
            *cmdState = 5;
        }
        break;
    case 5:
        *cmdState = 0;
        *b_rec = 0;
        if(checksum == Data)
        {
            platform_uart_protocol(cmd, rev_payload);
        }
        break;
    default:
        break;
    }
}

#endif  //USE_UART_SELF_PROTOCOL

#endif  //USE_UART_PROTOCOL

#if (USE_CHORD_MUSIC || USE_BUZZER)
static void power_on_delay(void)
{
    volatile tek_u32   delay_tick;
    delay_tick = sys_tick_get();
    while(sys_tick_get() - delay_tick < 200)
    {
#if USE_ZERO
        core_cal_zero(); //�ϵ��ȼ��һ�¹����źţ�û�����źŲ�����������ֻ����Ϊ��Щ��ϵ�ʱ���ѹ��λ���ϵ�ʱ�������������û�й���,�������������������Ǳ���
#endif
    }
}
#endif

void platform_init(void)
{
#if DEBUG_PRINT_ENABLE
    printf("App Start,platform init...\r\n");
    // printf("SDK version:%s\r\n", SDK_VERSION);
    printf("device version:%s\r\n", DEV_VERSION);
    printf("device model:%s\r\n", DEV_MODEL);
#endif
    SCD40_Init(); //SCD40传感器初始化
    SEN68_Init(); //SEN68传感器初始化
    OPT3004_Init(); //OPT3004传感器初始化

    encoder_init(); //绝对编码器初始化

#if USE_FAN_PWM
    tek_speed.table = speed_rpm;
    tek_speed.size = sizeof(speed_rpm) / 2;
    tek_speed.max = FAN_PWM_DUTY_MAX;
    tek_speed.fre = FAN_PWM_FRE;
    core_rpm_table_register(&tek_speed);
#endif
    core_tek_power_register();
    platform_power_up_init();
}

void core_drive_save(void);
void platformCycle(void)
{
#if USE_TE
    core_te_Handler();
#endif
#if USE_KEY
    core_KEY_main();
#endif
#if USE_FAN_PWM
    core_speed_handle();
#endif
    SCD40_Handler(); //SCD40传感器处理
    SEN68_Handler(); //SEN68传感器处理
    OPT3004_Handler(); //OPT3004传感器处理
#if HEPA_TIMEOUT
    core_hepa_handle();
#endif
#if USE_TIMER
    core_auto_off_handle();
#endif
#if USE_UART_PROTOCOL
    core_uart_protocol_Handler();
#endif
    core_drive_save();
}
