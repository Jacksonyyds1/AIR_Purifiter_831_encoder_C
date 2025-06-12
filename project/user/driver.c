#include "at32f403a_407_board.h"
#include "ihastek.h"
#include "flash.h"
#include "lcd.h"
#include "i2c.h"
#include "sen68.h"
#include "opt3004.h"

void driver_SCD40_UART_Send(unsigned char sendData) // TODO: uart接口给CO2传感器使用
{
    unsigned long uart2Tick;

    usart_data_transmit(UART5, sendData);
    uart2Tick = Get_Sys_Tick();
    while(!usart_flag_get(UART5, USART_TDC_FLAG) && (Get_Sys_Tick() - uart2Tick < 20));
    usart_flag_clear(UART5, USART_TDC_FLAG);
}

void driver_set_SCD40_Power(unsigned char onoff)
{
    if(onoff)
    {
        gpio_bits_set(CO2_SENSOR_EN_PORT, CO2_SENSOR_EN_PIN);
    }
    else
    {
        gpio_bits_reset(CO2_SENSOR_EN_PORT, CO2_SENSOR_EN_PIN);
    }
}

// TODO: 添加空气传感器数据读取函数

#if USE_UART_PROTOCOL
void drive_protocol_uart_send(unsigned char sendData)
{
    unsigned long uart3Tick;

    //UART_SendData(UART3,sendData);
    //  uart3Tick=Get_Sys_Tick();
    //  while((!UART_GetFlagStatus(UART3,UART_Flag_TX))&&(Get_Sys_Tick()-uart3Tick<20));
    //    UART_ClearFlag(UART3,UART_Flag_TX);
}
#endif

#if USE_FAN_PWM
void drive_pwm_duty_set(unsigned int duty)
{
    tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_2, duty);
    tmr_output_enable(TMR2, TRUE);
    tmr_counter_enable(TMR2, TRUE);
    //PWM_SetDuty(PWM0,PWM_Channel_3,duty);
    //TIM_PWMSetDuty(TIM7, TIM_PWMChannel_PWMA,duty); //ռ�ձ�����Ϊ50%//default 0
}
#endif

void drive_LCD_BackLight_set(uint16_t duty)
{
    unsigned int dutydata = duty * 180; //duty 0-100% to 0-18000

    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, dutydata);
    tmr_output_enable(TMR1, TRUE);
    tmr_counter_enable(TMR1, TRUE);
}

#define USERDATASAVEADDRESS     (FLASH_BANK1_START_ADDR+253*1024)

//error_status flash_write_word(uint32_t write_addr, uint32_t *p_buffer, uint16_t num_write);
void drive_dev_save_data(unsigned char *value, unsigned int len)
{
    flash_write_byte(USERDATASAVEADDRESS, value, len);
}

void drive_dev_read_data(unsigned char *value, unsigned int len)
{
    flash_read_byte(USERDATASAVEADDRESS, value, len);
}

void drive_set_ion(bool onoff)
{
    if(onoff)
    {
        gpio_bits_set(AIR_SENSOR_EN_PORT, AIR_SENSOR_EN_PIN);
    }
    else
    {
        gpio_bits_reset(AIR_SENSOR_EN_PORT, AIR_SENSOR_EN_PIN);
    }
}

void drive_set_uv(bool onoff)
{
    if(onoff)
    {
        gpio_bits_set(UV_EN_PORT, UV_EN_PIN);
    }
    else
    {
        gpio_bits_reset(UV_EN_PORT, UV_EN_PIN);
    }
}

void drive_set_scd40_power_up(bool enable)
{
    if(enable)
    {
        gpio_bits_set(CO2_SENSOR_EN_PORT, CO2_SENSOR_EN_PIN);
    }
    else
    {
        gpio_bits_reset(CO2_SENSOR_EN_PORT, CO2_SENSOR_EN_PIN);
    }
}

/**
  * @brief  Enable or disable SEN68 sensor power
  * @param  enable: true to enable, false to disable
  * @retval None
  */
void drive_sen68_power_up(bool enable)
{
    if(enable)
    {
        gpio_bits_set(AIR_SENSOR_EN_PORT, AIR_SENSOR_EN_PIN);
    }
    else
    {
        gpio_bits_reset(AIR_SENSOR_EN_PORT, AIR_SENSOR_EN_PIN);
    }
}

/****ENV DATA HANDLER END */

uint8_t drive_sen68_send_data(uint8_t *data, uint16_t len)
{
    return i2c2_send_data(SEN68_I2C_ADDRESS, data, len);
}

uint8_t drive_sen68_receive_data(uint8_t *data, uint16_t len)
{
    return i2c2_receive_data(SEN68_I2C_ADDRESS, data, len);
}

uint8_t drive_opt3004_send_data(uint8_t *data, uint16_t len)
{
    return i2c2_send_data(OPT3004_I2C_ADDRESS, data, len);
}

uint8_t drive_opt3004_receive_data(uint8_t *data, uint16_t len)
{
    return i2c2_receive_data(OPT3004_I2C_ADDRESS, data, len);
}
