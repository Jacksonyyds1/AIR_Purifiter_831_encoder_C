#ifndef __STEPPER_MOTOR_H__
#define __STEPPER_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f403a_407.h"
#include "stdbool.h"

// --- Stepper Motor Defines ---
#define STEP_TICKS_MAX 2000     // Maximum ticks for a step (2KHz control frequency)

typedef enum {
    Motor_State_Stop       = 0,
    Motor_State_Starting   = 1,
    Motor_State_Forward    = 2,
    Motor_State_Backward   = 3,
    Motor_State_Stopping   = 4
} Motor_State_t;

typedef enum {
    Motor_Running_Type_Positioning = 0,
    Motor_Running_Type_Direction   = 1
} Motor_Running_Type_t;

typedef enum {
    Motor_Direction_Forward  = 0,
    Motor_Direction_Backward = 1,
    Motor_Direction_Stop     = 2
} Motor_Direction_t;

Motor_State_t stepper_motor_get_state(uint8_t index);
Motor_Running_Type_t stepper_motor_get_running_type(uint8_t index);
int stepper_motor_get_position(uint8_t index);
void stepper_motor_set_target_position(uint8_t index, int position);
void stepper_motor_set_position(uint8_t index, int position);
void stepper_motor_set_direction(uint8_t index, Motor_Direction_t target_direction, uint16_t target_speed);
void stepper_motor_stop(uint8_t index, bool motor_break);
void stepper_motor_handler(void);

// --- Synchronous Control Functions ---
void stepper_motor_set_sync_target(int nozzle_target, int base_target, uint16_t speed);
bool stepper_motor_is_sync_complete(void);
bool stepper_motor_all_stopped(void);

// --- Configuration Functions ---
void stepper_motor_set_speed_profile(uint8_t index, uint16_t max_speed, uint8_t accel_rate);

#ifdef __cplusplus
}
#endif

#endif
