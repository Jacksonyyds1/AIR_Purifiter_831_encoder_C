#include "user_config.h"
#include "types.h"
#include "at32f403a_407_board.h"
#include "ihastek.h"
#include "display.h"

#if USE_TE

extern const char compile_time[];
static TEKTe_t  core_te;

static unsigned int  dev_addr;
static unsigned char  te_Tx_buf[TE_UART_TX_BUF_LEN];
static unsigned char  te_rx_buf[TE_UART_RX_BUF_LEN];
static TEKUart_t  te_uart;// =

unsigned char core_get_te_status(void)
{
    return core_te.close_debug;
}

TEKTe_t *core_get_TE(void)
{
    return &core_te;
}
#if USE_POWER
void core_tek_power_te_register(TEKPower_t *power)
{
    core_te.power = power;
}
#endif

#if USE_TIMER
void core_off_timer_te_register(TEKTime_t *time)
{
    core_te.time = time;
}
#endif
#if USE_PM25
void core_PM25_te_register(TEKPM25_t *pm25, TEKPM25_t *pm10)
{
    core_te.pm25 = pm25;
    core_te.pm10 = pm10;
}
#endif

#ifdef HEPA_TIMEOUT
void core_tek_hepa_te_register(TEKHepa_t *hepa)
{
    core_te.hepa = hepa;
}
#endif

#ifdef HEPA2_TIMEOUT
void core_tek_hepa2_te_register(TEKHepa_t *hepa2)
{
    core_te.hepa2 = hepa2;
}
#endif

#ifdef HEPA3_TIMEOUT
void core_tek_hepa3_te_register(TEKHepa_t *hepa3)
{
    core_te.hepa3 = hepa3;
}
#endif

#if USE_HUMIDIFIER
void core_himidity_te_register(TEKHumiditifier_t *humi)
{
    core_te.humi = humi;
}
#endif

#if USE_LED_TEST
void core_led_te_register(TEKLed_t *led)
{
    core_te.led = led;
}
#endif
#if (USE_BUZZER)
void core_buz_te_register(TEKBuzzer_t *buz)
{
    core_te.buz = buz;
}
#endif

#if USE_KEY
void core_key_te_register(TEKKey_t *key)
{
    core_te.key = key;
}
#endif

#if USE_FG
void core_fg_te_register(TEKFG_t *fg)
{
    core_te.fg = fg;
}
#endif

#if USE_ZERO
void core_zero_te_register(TEKZERO_t *zero)
{
    core_te.zero = zero;
}
#endif

#if USE_AIR_SENSOR
void core_air_te_register(TEKAir_t *air)
{
    core_te.air = air;
}
#endif

#if USE_FAN_PWM
void core_pwm_te_register(TEKPWM_t *pwm)
{
    core_te.pwm = pwm;
}
#endif

//#if USE_BURN_INFO
//void core_burn_info_te_register(void (*burn_info)(unsigned char *))
//{
//    core_te.burn_info = burn_info;
//}
//#endif

static void __core_dev_addr_data(unsigned int addr)
{
    unsigned char  B_EEPROMTEMP[8] = {0}, i;
    unsigned char sumcal = 0;
    B_EEPROMTEMP[0] = 0x5a;
    B_EEPROMTEMP[1] = addr & 0XFF;
    B_EEPROMTEMP[2] = (addr >> 8) & 0XFF;
    B_EEPROMTEMP[3] = 0x5a;
    sumcal = 0;
    for(i = 0; i < 7; i++)
    {
        sumcal += B_EEPROMTEMP[i];
    }
    B_EEPROMTEMP[7] = sumcal;
    // drive_dev_addr_save_data(B_EEPROMTEMP, 8);
}

void drive_te_uart_send(signed char c)
{
    unsigned long uart1Tick;

    usart_data_transmit(USART1, c);
    uart1Tick = Get_Sys_Tick();
    while(!usart_flag_get(USART1, USART_TDC_FLAG) && (Get_Sys_Tick() - uart1Tick < 20))
        ;
    usart_flag_clear(USART1, USART_TDC_FLAG);
}

static void core_uart_sent_to_TE(unsigned int len, unsigned char cmd, unsigned char *payload)
{
    unsigned char i, checksum;
    unsigned int all_len = len + 7;
    checksum = 0;

    if(all_len > TE_UART_TX_BUF_LEN)
    {
        return;
    }

    te_uart.ptx[0] = 0xc5;
    te_uart.ptx[1] = all_len & 0xff;
    te_uart.ptx[2] = (all_len >> 8) & 0xff;
    te_uart.ptx[3] = dev_addr & 0xff;
    te_uart.ptx[4] = (dev_addr >> 8) & 0xff;
    te_uart.ptx[5] = cmd;
    checksum = 0;
    for(i = 0; i < 6; i++)
    {
        checksum += te_uart.ptx[i];
    }
    for(i = 0; i < len; i++)
    {
        te_uart.ptx[i + 6] = payload[i];
        checksum += payload[i];
    }
    te_uart.ptx[i + 6] = checksum;
    for(i = 0; i < all_len; i++)
    {
        drive_te_uart_send(te_uart.ptx[i]);
    }
}

static unsigned char _core_isRecvBufferEmpty(void)
{
    if(te_uart.rx_rear == te_uart.rx_top)
    {
        return 1;
    }
    return 0;
}

void core_te_rec_byte(unsigned char value)
{
    te_uart.prx[te_uart.rx_rear++] = value;
    if(te_uart.rx_rear >= te_uart.rx_size)
    {
        te_uart.rx_rear = 0;
    }
}

static unsigned char _core_RecvBufferReadByte(void)
{
    unsigned char Data;
    Data = te_uart.prx[te_uart.rx_top];
    te_uart.rx_top = (te_uart.rx_top + 1) % te_uart.rx_size;
    return Data;
}

void _core_report_statue(unsigned char b_send)
{
    static unsigned char  status_array[30];
    unsigned char  send_pay[30] = {0};
    int i;
    send_pay[0] = core_te.power->on;
    //    if(core_te.power->lock) { send_pay[0] |= 1 << 1; }
    //    if(core_te.power->uv) { send_pay[0] |= 1 << 2; }
    //    if(core_te.power->ion) { send_pay[0] |= 1 << 3; }
    if(core_te.power->on &&     core_te.power->speed.speedpara[core_te.power->speed.changetype].mode == SPEEDMODE_AUTO)
    {
        send_pay[0] |= 1 << 4;
    }
    //    if(core_te.power->sleep) { send_pay[0] |= 1 << 5; }
    if(gpio_output_data_bit_read(AIR_SENSOR_EN_PORT, AIR_SENSOR_EN_PIN))
    {
        send_pay[0] |= 1 << 6;
    }

    if(core_te.power->on)
    {
        send_pay[1] = core_te.power->speed.speedpara[core_te.power->speed.changetype].lev + 1;
    }
    //    send_pay[1] |= core_te.power->night_step << 4;

    //    if(core_te.time != NULL)
    //    {
    //        send_pay[2] = core_te.time->step;
    //        send_pay[3] = core_te.time->left_secs & 0xff;
    //        send_pay[4] = (core_te.time->left_secs >> 8) & 0xff;
    //        send_pay[5] = (core_te.time->left_secs >> 16) & 0xff;
    //        send_pay[6] = (core_te.time->left_secs >> 24) & 0xff;
    //    }
    if(core_te.pm25 != NULL)
    {
        send_pay[7] = core_te.pm25->num & 0xff; //pm25
        send_pay[8] = (core_te.pm25->num >> 8) & 0xff;
    }
    send_pay[9] = (0 >> 16) & 0xff;
    send_pay[10] = (0 >> 24) & 0xff;
    if(core_te.hepa != NULL)
    {
        send_pay[11] = core_te.hepa->secs & 0xff; //
        send_pay[12] = (core_te.hepa->secs >> 8) & 0xff;
        send_pay[13] = (core_te.hepa->secs >> 16) & 0xff;
        send_pay[14] = (core_te.hepa->secs >> 24) & 0xff;
    }
#ifdef HEPA_TIMEOUT
    send_pay[15] = (HEPA_TIMEOUT / 3600) & 0xff;
    send_pay[16] = ((HEPA_TIMEOUT / 3600) >> 8) & 0xff;
#else
    send_pay[15] = 0;
    send_pay[16] = 0;
#endif
    send_pay[17] = 0;   //TODO: Error code
    send_pay[19] = core_te.pm10->num & 0xff; //pm10
    send_pay[20] = (core_te.pm10->num >> 8) & 0xff;

    send_pay[24] = core_te.time->left_secs & 0xff;
    send_pay[25] = (core_te.time->left_secs >> 8) & 0xff;
    send_pay[26] = (core_te.time->left_secs >> 16) & 0xff;
    send_pay[27] = (core_te.time->left_secs >> 24) & 0xff;

    send_pay[28] = core_get_tek_power()->rpmofspeed;
    send_pay[29] = (core_get_tek_power()->rpmofspeed >> 8);
    //    if(core_te.power->uv_fault || core_te.power->uv_fault2) { send_pay[17] |= 1 << 3; }
    for(i = 0; i < sizeof(send_pay); i++)
    {
        if(status_array[i] != send_pay[i] && core_te.auto_report)
        {
            b_send = 1;
        }
        status_array[i] = send_pay[i];
    }
    if(b_send)
    {
        core_uart_sent_to_TE(sizeof(send_pay), 0x3, send_pay);
    }
}

#if ((defined HEPA2_TIMEOUT) || (defined HEPA3_TIMEOUT ))
static void _core_report_hepa2_hepa3(unsigned char b_send)
{
    static unsigned char  status_array[12];
    unsigned char  send_pay[12] = {0};
    int i;
#ifdef HEPA2_TIMEOUT
    if(core_te.hepa2 != NULL)
    {
        send_pay[0] = core_te.hepa2->secs & 0xff; //
        send_pay[1] = (core_te.hepa2->secs >> 8) & 0xff;
        send_pay[2] = (core_te.hepa2->secs >> 16) & 0xff;
        send_pay[3] = (core_te.hepa2->secs >> 24) & 0xff;
    }
    send_pay[4] = (HEPA2_TIMEOUT / 3600) & 0xff;
    send_pay[5] = ((HEPA2_TIMEOUT / 3600) >> 8) & 0xff;
#endif
#ifdef HEPA3_TIMEOUT
    if(core_te.hepa3 != NULL)
    {
        send_pay[6] = core_te.hepa3->secs & 0xff; //
        send_pay[7] = (core_te.hepa3->secs >> 8) & 0xff;
        send_pay[8] = (core_te.hepa3->secs >> 16) & 0xff;
        send_pay[9] = (core_te.hepa3->secs >> 24) & 0xff;
    }
    send_pay[10] = (HEPA3_TIMEOUT / 3600) & 0xff;
    send_pay[11] = ((HEPA3_TIMEOUT / 3600) >> 8) & 0xff;
#endif
    for(i = 0; i < 12; i++)
    {
        if(status_array[i] != send_pay[i] && core_te.auto_report)
        {
            b_send = 1;
        }
        status_array[i] = send_pay[i];
    }
    if(b_send)
    {
        core_uart_sent_to_TE(12, 0x4, send_pay);
    }
}
#endif

#if USE_HUMIDIFIER
static void _core_report_humidifier(unsigned char b_send)
{
    static unsigned char  status_array[4];
    unsigned char  send_pay[4] = {0};
    int i;
    if(core_te.humi != NULL)
    {
        send_pay[0] = core_te.humi->low_water;
        if(core_te.humi->burn)
        {
            send_pay[0] |= 1 << 1;
        }
        if(core_te.humi->const_humi)
        {
            send_pay[0] |= 1 << 2;
        }
        if(core_te.humi->warm_fog)
        {
            send_pay[0] |= 1 << 3;
        }
        if(core_te.humi->sleep)
        {
            send_pay[0] |= 1 << 4;
        }
        if(core_te.humi->night)
        {
            send_pay[0] |= 1 << 5;
        }
        send_pay[1] = core_te.humi->const_humi_value;
        send_pay[2] = core_te.humi->humidity;
        send_pay[3] = core_te.humi->temperature;
    }
    for(i = 0; i < 4; i++)
    {
        if(status_array[i] != send_pay[i] && core_te.auto_report)
        {
            b_send = 1;
        }
        status_array[i] = send_pay[i];
    }
    if(b_send)
    {
        core_uart_sent_to_TE(4, 0x5, send_pay);
    }
}

#endif

static void _core_report_timer_hepa_on_secs(unsigned char b_send)
{
    static unsigned char  status_array[13];
    unsigned char  send_pay[13] = {0};
    int i;
    //    if(core_te.time != NULL)
    //    { send_pay[0] = core_te.time->hours; }
    if(core_te.hepa != NULL)
    {
        send_pay[1] = core_te.hepa->all_secs & 0xff;
        send_pay[2] = (core_te.hepa->all_secs >> 8) & 0xff;
        send_pay[3] = (core_te.hepa->all_secs >> 16) & 0xff;
        send_pay[4] = (core_te.hepa->all_secs >> 24) & 0xff;
        send_pay[5] = core_te.hepa->cnt & 0xff;
        send_pay[6] = (core_te.hepa->cnt >> 8) & 0xff;
        send_pay[7] = (core_te.hepa->cnt >> 16) & 0xff;
        send_pay[8] = (core_te.hepa->cnt >> 24) & 0xff;
    }
    send_pay[9] = core_te.power->on_secs & 0xff; //
    send_pay[10] = (core_te.power->on_secs >> 8) & 0xff;
    send_pay[11] = (core_te.power->on_secs >> 16) & 0xff;
    send_pay[12] = (core_te.power->on_secs >> 24) & 0xff;
    for(i = 0; i < 13; i++)
    {
        if(status_array[i] != send_pay[i] && core_te.auto_report)
        {
            b_send = 1;
        }
        status_array[i] = send_pay[i];
    }
    if(b_send)
    {
        core_uart_sent_to_TE(13, 0x6, send_pay);
    }
}

#if 1//USE_FG
static void _core_report_fg(unsigned char b_send)
{

#if USE_THREE_FG
    static unsigned char  status_array[9];
    unsigned char  send_pay[9] = {0};
#elif USE_TWO_FG
    static unsigned char  status_array[6];
    unsigned char  send_pay[6] = {0};
#else
    static unsigned char  status_array[3];
    unsigned char  send_pay[3] = {0};
#endif

    int i;
    // if(core_te.fg)
    {
        if(!core_te.auto_report)
        {
            send_pay[0] = gpio_input_data_bit_read(FG_PORT, FG_PIN);
        }
        send_pay[1] = /*core_te.fg->cnt*/core_get_tek_power()->rpmofspeed & 0xff;
        send_pay[2] = (/*core_te.fg->cnt*/core_get_tek_power()->rpmofspeed >> 8) & 0xff;
    }

#if USE_THREE_FG
    for(i = 0; i < 9; i++)
    {
        if(status_array[i] != send_pay[i] && core_te.auto_report)
        {
            b_send = 1;
        }
        status_array[i] = send_pay[i];
    }
    if(b_send)
    {
        core_uart_sent_to_TE(9, 0xb, send_pay);
    }

#elif USE_TWO_FG
    for(i = 0; i < 6; i++)
    {
        if(status_array[i] != send_pay[i] && core_te.auto_report)
        {
            b_send = 1;
        }
        status_array[i] = send_pay[i];
    }
    if(b_send)
    {
        core_uart_sent_to_TE(6, 0xb, send_pay);
    }

#else
    for(i = 0; i < 3; i++)
    {
        if(status_array[i] != send_pay[i] && core_te.auto_report)
        {
            b_send = 1;
        }
        status_array[i] = send_pay[i];
    }
    if(b_send)
    {
        core_uart_sent_to_TE(3, 0xb, send_pay);
    }
#endif
}
#endif

#if USE_UV
#if USE_UV_DETECT
static void _core_report_uv_detect(unsigned char b_send)
{
    static unsigned char  status_array[6];
    unsigned char  send_pay[6] = {0};
    int i;
    send_pay[0] = core_te.power->uv_detect & 0xff;
    send_pay[1] = Low_UV + 1;
    send_pay[2] = High_UV - 1;
    send_pay[3] = Low_UV;
    send_pay[4] = High_UV;
    send_pay[5] = (core_te.power->uv_detect >> 8) & 0xff;
    for(i = 0; i < 6; i++)
    {
        if(status_array[i] != send_pay[i] && core_te.auto_report)
        {
            b_send = 1;
        }
        status_array[i] = send_pay[i];
    }
    if(b_send)
    {
        core_uart_sent_to_TE(6, 0xC, send_pay);
    }
}

#endif
#endif

#if USE_AIR_SENSOR
static void _core_report_air(unsigned char b_send)
{
    static unsigned char  status_array[10];
    unsigned char  send_pay[10] = {0}, send_pay2[2];
    int i;
    if(core_te.air)
    {
        send_pay[0] = core_te.air->AD & 0xff;
        send_pay[1] = (core_te.air->AD >> 8) & 0xff;
        send_pay[2] = core_te.air->Ratio & 0xff;
        send_pay[3] = (core_te.air->Ratio >> 8) & 0xff;
        send_pay[4] = core_te.air->Es_Air_Condition;
        send_pay[5] = core_te.air->Rs & 0xff;
        send_pay[6] = (core_te.air->Rs >> 8) & 0xff;
        send_pay[7] = core_te.air->Ro & 0xff;
        send_pay[8] = (core_te.air->Ro >> 8) & 0xff;
        send_pay[9] = core_te.air->Air_Condition;
        send_pay2[0] = core_te.air->quantity_select;
    }
    for(i = 0; i < 10; i++)
    {
        if(status_array[i] != send_pay[i] && core_te.auto_report)
        {
            b_send = 1;
        }
        status_array[i] = send_pay[i];
    }
    if(b_send)
    {
        core_uart_sent_to_TE(10, 0x10, send_pay);
        core_uart_sent_to_TE(1, 0x1f, send_pay2);
    }
}
#endif

static void _core_report_pwm_buzzer_human(unsigned char b_send)
{
    static unsigned char  status_array[9];
    unsigned char  send_pay[9] = {0};
    int i;
    if(core_te.pwm)
    {
        send_pay[0] = /*core_te.pwm->max*/FAN_PWM_DUTY_MAX & 0xff; //pwm max
        send_pay[1] = /*(core_te.pwm->max >> 8)*/(FAN_PWM_DUTY_MAX >> 8) & 0xff;
        send_pay[2] = /*core_te.pwm->duty */core_tek_GetPWM()->duty & 0xff; //DUTY
        send_pay[3] = (/*core_te.pwm->duty*/core_tek_GetPWM()->duty >> 8) & 0xff;
        send_pay[7] = /*core_te.pwm->fre*/FAN_PWM_FRE & 0xff; // pwm fre
        send_pay[8] = /*(core_te.pwm->fre*/(FAN_PWM_FRE >> 8) & 0xff;
    }
    //    if(core_te.buz)
    //    {
    //        send_pay[4] = core_te.buz->fre & 0xff; //buzzer fre
    //        send_pay[5] = (core_te.buz->fre >> 8) & 0xff;
    //    }
    if(core_te.hepa)
    {
        send_pay[6] = core_te.hepa->human;
    }
#ifdef HEPA2_TIMEOUT
    if(core_te.hepa2 && core_te.hepa2->human)
    {
        send_pay[6] |= 1 << 1;
    }
#endif
#ifdef HEPA3_TIMEOUT
    if(core_te.hepa3 && core_te.hepa3->human)
    {
        send_pay[6] |= 1 << 2;
    }
#endif
    for(i = 0; i < 9; i++)
    {
        if(status_array[i] != send_pay[i] && core_te.auto_report)
        {
            b_send = 1;
        }
        status_array[i] = send_pay[i];
    }
    if(b_send)
    {
        core_uart_sent_to_TE(9, 0x11, send_pay);
    }
}

static void _core_report_pm25_step(unsigned char b_send)
{
    static unsigned char  status_array[2];
    unsigned char  send_pay[2] = {0};
    int i;
    // TODO: 根据新的传感器上报数据
    if(core_te.pm25)
    {
        send_pay[0] = core_te.pm25->force;
        send_pay[1] = core_te.pm25->step;
    }
    for(i = 0; i < 2; i++)
    {
        if(status_array[i] != send_pay[i] && core_te.auto_report)
        {
            b_send = 1;
        }
        status_array[i] = send_pay[i];
    }
    if(b_send)
    {
        core_uart_sent_to_TE(2, 0x1e, send_pay);
    }
}

#if USE_PM25
#if (USE_PPD42 ||  USE_PM25)
static void   _core_report_ppd42_per_value(unsigned char b_send)
{
    static unsigned char  status_array[3];
    unsigned char  send_pay[3] = {0};
    int i;
    if(core_te.pm25)
    {
        send_pay[0] = core_te.pm25->ppd42_per;
        if(core_get_tek_power()->displaymode == DISPLAYMODE_PM25PAGE)
        {
            send_pay[1] = core_te.pm25->ram & 0xff;
            send_pay[2] = (core_te.pm25->ram >> 8) & 0xff;
        }
        else
        {
            send_pay[1] = core_te.pm10->ram & 0xff;
            send_pay[2] = (core_te.pm10->ram >> 8) & 0xff;
        }
    }
    for(i = 0; i < 3; i++)
    {
        if(status_array[i] != send_pay[i] && core_te.auto_report)
        {
            b_send = 1;
        }
        status_array[i] = send_pay[i];
    }
    if(b_send)
    {
        core_uart_sent_to_TE(3, 0x20, send_pay);
    }
}
#endif
#endif

#if USE_STATUS_WRITE
unsigned long core_get_status_write_cnt(void);
static void _core_report_status_write_cnt_value(unsigned char b_send)
{
    static unsigned char  status_array[4];
    unsigned char  send_pay[4] = {0};
    int i;

    send_pay[0] = core_get_status_write_cnt() & 0xff;
    send_pay[1] = (core_get_status_write_cnt() >> 8) & 0xff;
    send_pay[2] = (core_get_status_write_cnt() >> 16) & 0xff;
    send_pay[3] = (core_get_status_write_cnt() >> 24) & 0xff;

    for(i = 0; i < 4; i++)
    {
        if(status_array[i] != send_pay[i] && core_te.auto_report)
        {
            b_send = 1;
        }
        status_array[i] = send_pay[i];
    }
    if(b_send)
    {
        core_uart_sent_to_TE(4, 0x22, send_pay);
    }
}

#endif
static unsigned int  b_te_init;
static void __core_te_init(void)
{

    if(!b_te_init && Get_Sys_Tick() > 500)
    {
        unsigned char  value[8] = {0}, sum = 0, i;
        // drive_dev_addr_read_data(value, 8);
        for(i = 0; i < 7; i++)
        {
            sum += value[i];
        }
        if((sum == value[7]) && (value[0] == 0x5a))
        {
            dev_addr = (unsigned int)value[1] | (unsigned int)value[2] << 8;
        }
        te_uart.prx = te_rx_buf;
        te_uart.ptx = te_Tx_buf;
        te_uart.rx_size = TE_UART_RX_BUF_LEN;
        te_uart.tx_size = TE_UART_TX_BUF_LEN;
        te_uart.rx_rear = 0;
        te_uart.rx_top = 0;
        te_uart.tx_rear = 0;
        te_uart.tx_top = 0;
        b_te_init = 1;
    }
}

void core_te_set_hepa(unsigned long hepa_s);
void core_te_Handler(void)
{
    unsigned char Data;
    static unsigned char    cmdState, checksum, cmd;
    static unsigned char    b_rec;
    static unsigned long   rec_tick;
    static unsigned char    palyload_index;
    static unsigned int   rev_len, rev_addr;
    static unsigned char    rev_payload[30];
    //    unsigned char  send_pay[30] = {0};
    __core_te_init();
    while(!_core_isRecvBufferEmpty())
    {
        Data = _core_RecvBufferReadByte();
        b_rec = 1;
        rec_tick = Get_Sys_Tick();
        switch(cmdState)
        {
        case 0:
            if(Data == 0xC5)
            {
                checksum = Data;
                cmdState = 1;
            }
            else
            {
                b_rec = 0;
                te_uart.rx_rear = te_uart.rx_top = 0;
            }
            break;
        case 1:
            rev_len = Data;
            cmdState = 2;
            checksum += Data;
            break;
        case 2:
            rev_len |= (unsigned int)Data << 8;
            cmdState = 3;
            checksum += Data;
            if(rev_len - 7 > 30) //rev_payload[30]; 不能超过rev_payload[30]下标
            {
                cmdState = 0;
                b_rec = 0;
                te_uart.rx_rear = te_uart.rx_top = 0;
            }
            break;
        case 3:               //addr
            rev_addr = Data;
            cmdState = 4;
            checksum += Data;
            break;
        case 4:               //addr
            rev_addr |= (unsigned int)Data << 8;
            cmdState = 5;
            checksum += Data;
            break;
        case 5:
            cmd = Data;
            checksum += Data;
            if(rev_len > 7)
            {
                cmdState = 6;
            }
            else
            {
                cmdState = 7;
            }
            palyload_index = 0;
            break;
        case 6:
            if(palyload_index >= 30)  //rev_payload[30]; 不能超过rev_payload[0]下标
            {
                palyload_index = 0;
                cmdState = 0;
                b_rec = 0;
                te_uart.rx_rear = te_uart.rx_top = 0;
                break;
            }
            rev_payload[palyload_index++] = Data;
            checksum += Data;
            if(palyload_index >= (rev_len - 7))
            {
                cmdState = 7;
            }
            break;
        case 7:
            cmdState = 0;
            b_rec = 0;
            if(checksum == Data)
            {
                if((dev_addr == 0) && b_te_init)
                {
                    dev_addr = Get_Sys_Tick() % 65535;
                    __core_dev_addr_data(dev_addr);
                }
                if((rev_addr == 0) || (rev_addr == 0xffff) || (rev_addr == dev_addr))
                {
                    core_te.close_debug = 1;
                    switch(cmd)
                    {
                    case 0x1:
                    {
                        core_uart_sent_to_TE(sizeof(DEV_VERSION), 0x1, (unsigned char *)DEV_VERSION);
                    }
                    break;
                    case 0x2:
                    {
                        core_uart_sent_to_TE(sizeof(DEV_MODEL), 0x2, (unsigned char *)DEV_MODEL);
                    }
                    break;
                    case 0x3:
                    {
                        _core_report_statue(1);
                    }
                    break;
#if ((defined HEPA2_TIMEOUT) || (defined HEPA3_TIMEOUT ))
                    case 0x4://HEPA2,HEPA3
                    {
                        _core_report_hepa2_hepa3(1);
                    }
                    break;
#endif
#if USE_HUMIDIFIER
                    case 0x5://humidifier
                    {
                        _core_report_humidifier(1);
                    }
                    break;
#endif
                    case 0x6:
                    {
                        _core_report_timer_hepa_on_secs(1);
                    }
                    break;
                    case 0x7:
                    {
                        switch(rev_payload[0])
                        {
                        case 1:
                            core_te.power->on = 1;
                            //core_te.power->speed = 1;
                            break;
                        case 2:
                            core_te.power->on = 0;
                            break;
                        // case 3:
                        //     if(core_te.power->on) { core_te.power->night_step = 1; }
                        //     core_te.power->sleep = 1;
                        //     break;
                        // case 4:
                        //     core_te.power->night_step = 0;
                        //     core_te.power->sleep = 0;
                        //     break;
                        case 5:
                            //if(core_te.power->on) { core_te.power->aut = 1; }
                            break;
                        case 6:
                            //core_te.power->aut = 0;
                            break;
                        case 10:
                            core_te.power->speed.speedpara[core_te.power->speed.changetype].lev = (SPEEDLEVEL)rev_payload[1];
                            break;
                        case 11:
                            break;
                        case 12:
                            break;
                        case 13:
                            break;
#if USE_UV
                        case 14:
                            if(core_te.power->on)
                            {
                                core_te.power->uv = 1;
                            }
                            break;
                        case 15:
                            core_te.power->uv = 0;
                            break;  //uv off
#endif
#if USE_ION
                        case 16:
                            if(core_te.power->on)
                            {
                                core_te.power->ion = 1;
                            }
                            break;  //ion on
                        case 17:
                            core_te.power->ion = 0;
                            break;  //ion off
#endif
#if USE_LOCK
                        case 18:
                            if(core_te.power->on)
                            {
                                core_te.power->lock = 1;
                            }
                            break;  //lock on
                        case 19:
                            core_te.power->lock = 0;
                            break;  //lock off
#endif
                        case 20:
                            if(core_te.pm25 != NULL)
                            {
                                core_te.pm25->on = 1;
                            }
                            break; //pm2.5 on
                        case 21:
                            if(core_te.pm25 != NULL)
                            {
                                core_te.pm25->on = 0;
                            }
                            break;  //pm2.5 off
                        // case 22:
                        // {
                        //     if(core_te.power->on && (core_te.time != NULL))
                        //     {
                        //         core_te.time->step = rev_payload[1];
                        //     }
                        // }
                        // break;
                        // case 23:
                        //     if(core_te.time != NULL) { core_te.time->step = 0; }
                        //     break;
                        // case 24:
                        //     if(core_te.power->on) { core_te.power->night_step = 1; }
                        //     break;
                        // case 25:
                        //     core_te.power->night_step = 0;
                        //     break;
                        // case 26:
                        //     if(core_te.led != NULL) { core_te.led->all_on = 1; }
                        //     break;
                        // case 27:
                        //     if(core_te.led != NULL) { core_te.led->all_on = 0; }
                        //     break;
#if USE_BUZZER || USE_CHORD_MUSIC
                        case 28:
                        {
                            extern void core_buzzer_on_off(unsigned int onoff);
                            core_buzzer_on_off(1);
                        }
                        break;  //buzzer on
                        case 29:
                        {
                            extern void core_buzzer_on_off(unsigned int onoff);
                            core_buzzer_on_off(0);
                        }
                        break;  //buzzer off
#endif
                        case 30:
                            break;//key test in
                        case 31:
                            break;//key test out
                        case 34:
                        {
                            if(core_te.key)
                            {
                                core_uart_sent_to_TE(2, 0x8, (unsigned char *)&core_te.key->te);
                            }
                        }
                        break;
                        default:
                            break;
                        }
                    }
                    break;
                    case 0xa:
                    {
                        //TODO: LCD TE control
                    }
                    break;
#if 1//USE_FG
                    case 0xb://fg
                    {
                        _core_report_fg(1);
                    }
                    break;
#endif
#if USE_UV
#if USE_UV_DETECT
                    case 0xc://uv
                    {
                        _core_report_uv_detect(1);
                    }
                    break;
#endif
#endif
#ifdef USE_ZERO
                    case 0xe://zero
                    {
                        if(core_te.zero)
                        {
                            send_pay[0] = core_te.zero->pin;
                            send_pay[1] = core_te.zero->fre;
                            core_uart_sent_to_TE(2, 0xe, send_pay);
                        }
                    }
                    break;
#endif
#if USE_COMPILE_TIME
                    case 0xf:  //Compiler_Time
                    {
#ifdef USER_COMPILE_TIME
                        core_uart_sent_to_TE(sizeof(USER_COMPILE_TIME), 0xf, (unsigned char *)USER_COMPILE_TIME);
#else
                        CONST_T char   CODE_T Compiler_Time[] = __DATE__ " "__TIME__;
                        core_uart_sent_to_TE(sizeof(Compiler_Time), 0xf, (unsigned char *)Compiler_Time);
#endif

                    }
                    break;
#endif
                        //                            case 0xf:  //Compiler_Time
                        //                            {
                        //                                core_uart_sent_to_TE(21, 0xf, (unsigned char *)compile_time);
                        //                            }
                        //                            break;
#if USE_AIR_SENSOR
                    case 0x10://air
                    {
                        _core_report_air(1);
                    }
                    break;
#endif
                    case 0x11:
                    {
                        _core_report_pwm_buzzer_human(1);
                    }
                    break;
#if USE_FAN_PWM
                    case 0x13:
                    {

#if 1//USE_FG   //FG
                        void drive_pwm_duty_set(unsigned int duty);
                        //extern void core_te_set_speed(unsigned int speed);
#else
                        extern void drive_pwm_duty_set(unsigned int duty);

#endif
                        unsigned int duty;
                        duty = (unsigned int)rev_payload[0] | ((unsigned int)rev_payload[1] << 8);
                        core_tek_GetPWM()->duty = duty;
#if  0//USE_FG   //FG
                        core_te_set_speed(duty);

#else

                        drive_pwm_duty_set(duty);
                        if(core_te.pwm)
                        {
                            core_te.pwm->duty = duty;
                            core_te.pwm->te_set = 1;
                        }
#endif
                    }
                    break;
#endif
#ifdef HEPA_TIMEOUT
                    case 0x15:
                    {
                        unsigned long hepa;
                        hepa = (unsigned long)rev_payload[0] | ((unsigned long)rev_payload[1] << 8) | ((unsigned long)rev_payload[2] << 16) | ((unsigned long)rev_payload[3] << 24);
                        core_te_set_hepa(hepa);
                    }
                    break;
#endif
#ifdef HEPA2_TIMEOUT
                    case 0x16:
                    {
                        unsigned long hepa;
                        hepa = (unsigned long)rev_payload[0] | ((unsigned long)rev_payload[1] << 8) | ((unsigned long)rev_payload[2] << 16) | ((unsigned long)rev_payload[3] << 24);
                        core_te_set_hepa2(hepa);
                    }
                    break;
#endif
#ifdef HEPA3_TIMEOUT
                    case 0x17:
                    {
                        unsigned long hepa;
                        hepa = (unsigned long)rev_payload[0] | ((unsigned long)rev_payload[1] << 8) | ((unsigned long)rev_payload[2] << 16) | ((unsigned long)rev_payload[3] << 24);
                        core_te_set_hepa3(hepa);
                    }
                    break;
#endif
                    case 0x19:
                    {
                        core_uart_sent_to_TE(sizeof(DOC_VERSION), 0x19, (unsigned char *)DOC_VERSION);
                    }
                    break;
                    case 0x1a:
                    {
                        core_uart_sent_to_TE(sizeof(HW_VERSION), 0x1a, (unsigned char *)HW_VERSION);
                    }
                    break;
                    case 0x1c:
                    {
                        core_te.auto_report = rev_payload[0];
                    }
                    break;
#if USE_PM25
                    case 0x1e:
                    {
                        _core_report_pm25_step(1);
                    }
                    break;
#endif
#if USE_AIR_SENSOR
                    case 0x1f:
                    {
                        if(core_te.air)
                        {
                            core_te.air->quantity_select = rev_payload[0];
                            core_uart_sent_to_TE(1, 0x1f, (unsigned char *)&core_te.air->quantity_select);
                        }
                    }
                    break;
#endif
#if USE_PM25
                    case 0x20:
                    {
                        _core_report_ppd42_per_value(1);
                    }
                    break;
#endif
                    case 0x21://修改设备地址
                    {
                        dev_addr = rev_payload[0] << 8 | (rev_payload[1]);
                        __core_dev_addr_data(dev_addr);
                    }
                    break;
#if USE_STATUS_WRITE
                    case 0x22:
                    {
                        _core_report_status_write_cnt_value(1);
                    }
                    break;
#endif
#if USE_TWO_FG || USE_THREE_FG
                    case 0x23:
                    {
                        _core_report_fg2_pwm(1);
                    }
                    break;
#endif
                    case 0x30:
                    {
                    }
                    break;
                    default:
                        break;
                    }
                }
            }
            break;
        default:
            break;
        }
    }
    if(b_rec && (Get_Sys_Tick() - rec_tick > 600))
    {
        b_rec = 0;
        cmdState = 0;
        palyload_index = 0;
    }

    //以下自动判断是否上报数据
    _core_report_statue(0);
#if ((defined HEPA2_TIMEOUT) || (defined HEPA3_TIMEOUT ))
    _core_report_hepa2_hepa3(0);
#endif
#if USE_HUMIDIFIER
    _core_report_humidifier(0);
#endif
    _core_report_timer_hepa_on_secs(0);
#if USE_FG
    _core_report_fg(0);
#endif
#if USE_UV
#if USE_UV_DETECT
    _core_report_uv_detect(0);
#endif
#endif
#if USE_AIR_SENSOR
    _core_report_air(0);
#endif
    _core_report_pwm_buzzer_human(0);
    _core_report_pm25_step(0);
#if USE_STATUS_WRITE
    _core_report_status_write_cnt_value(0);
#endif

#if USE_THREE_FG || USE_TWO_FG
    _core_report_fg2_pwm(0);
#endif

}

#endif  //USE_TE
