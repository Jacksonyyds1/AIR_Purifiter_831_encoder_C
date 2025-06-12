#ifndef DRIVER_H
#define DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "user_config.h"

// CO2传感器相关函数
void driver_SCD40_UART_Send(unsigned char sendData);
void driver_set_SCD40_Power(unsigned char onoff);

// 协议通信函数
#if USE_UART_PROTOCOL
void drive_protocol_uart_send(unsigned char sendData);
#endif

// PWM控制函数
#if USE_FAN_PWM
void drive_pwm_duty_set(unsigned int duty);
#endif
    
// LCD背光控制函数
void drive_LCD_BackLight_set(uint16_t duty);

// 数据存储函数
void drive_dev_save_data(unsigned char *value, unsigned int len);
void drive_dev_read_data(unsigned char *value, unsigned int len);

// 设备控制函数
void drive_set_ion(bool onoff);
void drive_set_uv(bool onoff);
void drive_sen68_power_up(bool enable);
void drive_set_scd40_power_up(bool enable);

// I2C通信函数
uint8_t drive_sen68_send_data(uint8_t *data, uint16_t len);
uint8_t drive_sen68_receive_data(uint8_t *data, uint16_t len);
uint8_t drive_opt3004_send_data(uint8_t *data, uint16_t len);
uint8_t drive_opt3004_receive_data(uint8_t *data, uint16_t len);

#endif // DRIVER_H