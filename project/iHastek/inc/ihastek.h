#ifndef __IHASTEK_H
#define __IHASTEK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "user_config.h"
#include "types.h"
#include "type.h"
#include "define.h"

extern volatile unsigned int sys_tick,system_tick;

#define Set_Sys_TickInc()  sys_tick++
#define Set_System_TickInc()  system_tick++

unsigned int Get_Sys_Tick(void);
unsigned int Get_System_Tick(void);

void platform_init(void);
void platformCycle(void);


TEKPower_t *core_get_tek_power(void);
void core_power_status(void);
void core_tek_power_register(void);

#if USE_FAN_FG
extern unsigned int fg_cn_tem;
#define HAL_FG_CNT_INC()    fg_cn_tem++
#endif

#if UV_FREQUENCE_DETECT
UV_TypeDef *core_get_uv(void);
//void core_uv_err_counter(void);
#endif


#if USE_TM1668
void platform_tm1668_display(tek_u8 *value);
#endif

#if USE_TIMER
void platform_auto_off(void);
void core_set_auto_off_step(unsigned char step, bool is_self_increase);
#endif


#if USE_UART_PROTOCOL
void core_uart_protocol_sent(unsigned int len, unsigned char cmd, unsigned char *payload);
void core_uart_protocol_Handler(void);
#endif
#if USE_KEY
uint8_t core_get_key_value(void);
void core_key_release(uint16_t key);
void core_key_long_press(uint16_t key);
void core_KEY_main(void);
#endif
#if USE_FAN_PWM
void core_rpm_table_register(TEKSpeed_t *tek_speed);
TEKPWM_t *core_tek_GetPWM(void);
void core_speed_handle(void);
#endif
#if (USE_BUZZER || USE_CHORD_MUSIC)
void core_buzz_drive(void);
#endif
#if USE_UV
void core_uv_handle(void);
#endif
#if USE_TM1668
void core_TM_main(void);
#endif
#ifdef HEPA_TIMEOUT
void core_hepa_handle(void);
void core_hepa_reset(void);
void core_hepa2_reset(void);
TEKHepa_t *core_get_hepa(void);
TEKHepa_t *core_get_hepa2(void);
void drive_hepa_read_data(TEKFlash_e type, unsigned char *value, unsigned char len);
void drive_hepa_save_data(TEKFlash_e type, unsigned char *value, unsigned char len);
#endif
//#if(USE_HEPA || SAVE_USERDATA_ENABLE)
//void core_save_userData(void);
//#endif
#if USE_TE
TEKTe_t *core_get_TE(void);
void drive_te_uart_send(signed char c);
void core_te_set_hepa2(unsigned long hepa_s);
void core_tek_power_te_register(TEKPower_t *power);
void core_te_rec_byte(unsigned char value);
void core_te_Handler(void);
void core_pwm_te_register(TEKPWM_t *pwm);
void core_key_te_register(TEKKey_t *key);
#ifdef HEPA_TIMEOUT
void core_tek_hepa_te_register(TEKHepa_t *hepa);
#endif
#ifdef HEPA2_TIMEOUT
void core_tek_hepa2_te_register(TEKHepa_t *hepa2);
#endif
#endif
#ifdef USE_TIMER
TEKTime_t *core_get_tek_timer(void);
void core_auto_off_handle(void);
void core_off_timer_te_register(TEKTime_t *time);
void core_off_timer_table_register(unsigned char *table, unsigned char table_size);
#endif

void drive_pwm_duty_set(unsigned int duty);
#if USE_STATUS_WRITE
void UserFactoryReset(void);
#endif
#ifdef __cplusplus
}
#endif
#endif
