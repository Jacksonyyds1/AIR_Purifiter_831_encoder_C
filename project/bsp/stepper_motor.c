#include "ihastek.h"
#include "define.h"
#include "stepper_motor.h"
#include "encoder.h"

typedef struct
{
    Motor_State_t state;
    Motor_Running_Type_t running_type;
    
    bool motor_break;
    
    uint8_t accelerate_rate;
    uint16_t accelerate_step;
    Motor_Direction_t current_direction;
    Motor_Direction_t target_direction;

    int position;
    int target_position;

    uint16_t step_ticks;
    uint16_t current_speed;
    uint16_t target_speed;
} stepper_motor_t;

static stepper_motor_t stepper_motor[2] = {
    {
        Motor_State_Stop, Motor_Running_Type_Positioning, 
        0, 
        100, 0, Motor_Direction_Stop, Motor_Direction_Stop,
        0, 0, 
        STEP_TICKS_MAX, 0, 0
    }, // MOTOR_NOZZLE
    {
        Motor_State_Stop, Motor_Running_Type_Positioning, 
        0, 
        100, 0, Motor_Direction_Stop, Motor_Direction_Stop,
        0, 0, 
        STEP_TICKS_MAX, 0, 0
    }, // MOTOR_BASE
};

//引脚顺序：A+ B+ A- B-
static const uint16_t unipolar_step_sequence[8] =
{
    0b0001,
    0b0011,
    0b0010,
    0b0110,
    0b0100,
    0b1100,
    0b1000,
    0b1001
};

static void set_stepper_motor_output(uint8_t index, int position, uint8_t motor_break)
{
    uint16_t port_val;
    if(index == MOTOR_NOZZLE)
    {
        port_val = gpio_output_data_read(MOTOR_NOZZLE_PORT) & 0xFF0F;
        if(motor_break)
        {
            port_val |= unipolar_step_sequence[position & 0x07] << 4;
            gpio_port_write(MOTOR_NOZZLE_PORT, port_val);
        }
        else
        {
            gpio_port_write(MOTOR_NOZZLE_PORT, port_val);
        }
    }
    else if(index == MOTOR_BASE)
    {
        port_val = gpio_output_data_read(MOTOR_BASE_PORT) & 0xF0FF;
        if(motor_break)
        {
            port_val |= unipolar_step_sequence[position & 0x07] << 8;
            gpio_port_write(MOTOR_BASE_PORT, port_val);
        }
        else
        {
            gpio_port_write(MOTOR_BASE_PORT, port_val);
        }
    }
}

uint16_t stepper_motor_speed_handler(uint16_t current_speed, uint16_t target_speed, uint8_t accelerate_rate)
{
    if (current_speed < target_speed) // current speed is slower than target
    {
        // Accelerate
        if (target_speed - current_speed <= accelerate_rate) 
        {
            current_speed = target_speed; // Max speed or clamp
        }
        else
        {
            current_speed += accelerate_rate;
        }
    }
    else if (current_speed > target_speed) // current speed is faster than target
    {
        // Decelerate
        if (current_speed - target_speed <= accelerate_rate) 
        {
            current_speed = target_speed; // Min speed or clamp
        }
        else
        {
            current_speed -= accelerate_rate;
        }
    }
    else
    {
        current_speed = target_speed;
    }

    return current_speed;
}

void stepper_motor_position_handler(stepper_motor_t *motor)
{
    uint16_t target_speed = 0;
    switch(motor->state)
    {
    case Motor_State_Stop:
        if(motor->position != motor->target_position)
        {
            motor->state = Motor_State_Starting;
            motor->accelerate_step = 0;
            motor->current_speed = 0;
        }
        break;

    case Motor_State_Starting:
        if(motor->position < motor->target_position)
        {
            motor->position++;
            motor->state = Motor_State_Forward;
            motor->current_direction = Motor_Direction_Forward;
        }
        else if(motor->position > motor->target_position)
        {
            motor->position--;
            motor->state = Motor_State_Backward;
            motor->current_direction = Motor_Direction_Backward;
        }

        target_speed = motor->target_speed;
        break;

    case Motor_State_Forward:
        motor->position++;
        if (motor->position < motor->target_position)
        {
            if ((motor->target_position - motor->position) <= motor->accelerate_step)
            {
                motor->state = Motor_State_Stopping;
            }
            else
            {
                target_speed = motor->target_speed;
            }
        }
        else
        {
            motor->state = Motor_State_Stopping;
        }
        break;

    case Motor_State_Backward:
        motor->position--;
        if (motor->position > motor->target_position)
        {
            if ((motor->position - motor->target_position) <= motor->accelerate_step)
            {
                motor->state = Motor_State_Stopping;
            }
            else
            {
                target_speed = motor->target_speed;
            }
        }
        else
        {
            motor->state = Motor_State_Stopping;
        }
        break;

    case Motor_State_Stopping:
        if(motor->accelerate_step)
        {
            if(motor->current_direction == Motor_Direction_Forward)
            {
                motor->position++;
            }
            else
            {
                motor->position--;
            }
        }
        else
        {
            motor->state = Motor_State_Stop;
            motor->current_speed = 0;
            motor->accelerate_step = 0;
        }
        break;

    default:
        break;
    }

    if (target_speed > motor->current_speed)        // Accelerate
    {
        motor->accelerate_step++;
        motor->current_speed = stepper_motor_speed_handler(motor->current_speed, target_speed, motor->accelerate_rate);
    }
    else if (target_speed < motor->current_speed)   // Decelerate
    {
        motor->accelerate_step--;
        motor->current_speed = stepper_motor_speed_handler(motor->current_speed, target_speed, motor->accelerate_rate);
    }
}

void stepper_motor_direction_handler(stepper_motor_t *motor)
{
    uint16_t target_speed = 0;
    switch(motor->target_direction)
    {
    case Motor_Direction_Stop:
        if(motor->current_direction == Motor_Direction_Forward)
        {
            motor->position++;
        }
        else if(motor->current_direction == Motor_Direction_Backward)
        {
            motor->position--;
        }

        if(motor->accelerate_step)
        {
            motor->state = Motor_State_Stopping;
        }
        else
        {
            motor->state = Motor_State_Stop;
            motor->current_direction = Motor_Direction_Stop;
            motor->current_speed = 0;
            motor->accelerate_step = 0;
        }
        break;

    case Motor_Direction_Forward:
        if(motor->current_direction == Motor_Direction_Backward)
        {
            if(motor->accelerate_step == 0)
            {
                motor->current_direction = Motor_Direction_Stop;
                motor->state = Motor_State_Stop;
            }
            else
            {
                motor->state = Motor_State_Stopping;
                motor->position--;
            }
        }
        else
        {
            motor->position++;
            motor->state = Motor_State_Forward;
            motor->current_direction = Motor_Direction_Forward;
            target_speed = motor->target_speed;
        }
        break;
    
    case Motor_Direction_Backward:
        if(motor->current_direction == Motor_Direction_Forward)
        {
            if(motor->accelerate_step == 0)
            {
                motor->current_direction = Motor_Direction_Stop;
                motor->state = Motor_State_Stop;
            }
            else
            {
                motor->state = Motor_State_Stopping;
                motor->position++;
            }
        }
        else
        {
            motor->position--;
            motor->state = Motor_State_Backward;
            motor->current_direction = Motor_Direction_Backward;
            target_speed = motor->target_speed;
        }
        break;
    }

    if (target_speed > motor->current_speed)        // Accelerate
    {
        motor->accelerate_step++;
        motor->current_speed = stepper_motor_speed_handler(motor->current_speed, target_speed, motor->accelerate_rate);
    }
    else if (target_speed < motor->current_speed)   // Decelerate
    {
        motor->accelerate_step--;
        motor->current_speed = stepper_motor_speed_handler(motor->current_speed, target_speed, motor->accelerate_rate);
    }
}

void stepper_motor_driver(uint8_t index)
{
    stepper_motor_t *motor = &stepper_motor[index];

    if (motor->running_type == Motor_Running_Type_Positioning)
    {
        stepper_motor_position_handler(motor);
    }
    else
    {
        stepper_motor_direction_handler(motor);
    }

    if (motor->state == Motor_State_Stop)
    {
        set_stepper_motor_output(index, motor->position, motor->motor_break);
    }
    else
    {
        set_stepper_motor_output(index, motor->position, 1);
        encoder_input_position(index, (int)motor->current_direction, 1);
    }
}

Motor_State_t stepper_motor_get_state(uint8_t index)
{
    return stepper_motor[index].state;
}

Motor_Running_Type_t stepper_motor_get_running_type(uint8_t index)
{
    return stepper_motor[index].running_type;
}

void stepper_motor_set_position(uint8_t index, int position)
{
    stepper_motor[index].position = position;
    stepper_motor[index].target_position = position;
    stepper_motor[index].state = Motor_State_Stop;
}

int stepper_motor_get_position(uint8_t index)
{
    return stepper_motor[index].position;
}

void stepper_motor_set_target_position(uint8_t index, int target_position)
{
    stepper_motor[index].target_position = target_position;
    stepper_motor[index].running_type = Motor_Running_Type_Positioning;
}

void stepper_motor_stop(uint8_t index, bool motor_break)
{
    stepper_motor[index].motor_break = motor_break;
    stepper_motor[index].state = Motor_State_Stopping;
    stepper_motor[index].target_direction = Motor_Direction_Stop;
}

void stepper_motor_set_direction(uint8_t index, Motor_Direction_t target_direction, uint16_t target_speed)
{
    stepper_motor[index].running_type = Motor_Running_Type_Direction;
    stepper_motor[index].target_direction = target_direction;
    stepper_motor[index].target_speed = target_speed;
}

void stepper_motor_handler(void)
{
    if(stepper_motor[MOTOR_NOZZLE].step_ticks == 0)
    {
        stepper_motor_driver(MOTOR_NOZZLE);
        stepper_motor[MOTOR_NOZZLE].step_ticks = STEP_TICKS_MAX - stepper_motor[MOTOR_NOZZLE].current_speed;
    }
    else
    {
        // Decrement step ticks
        stepper_motor[MOTOR_NOZZLE].step_ticks--;
    }
    
    if(stepper_motor[MOTOR_BASE].step_ticks == 0)
    {
        stepper_motor_driver(MOTOR_BASE);
        stepper_motor[MOTOR_BASE].step_ticks = STEP_TICKS_MAX - stepper_motor[MOTOR_BASE].current_speed;
    }
    else
    {
        // Decrement step ticks
        stepper_motor[MOTOR_BASE].step_ticks--;
    }
}

// --- Synchronous Control Functions ---
void stepper_motor_set_sync_target(int nozzle_target, int base_target, uint16_t speed)
{
    // Set both motors to positioning mode with same speed
    stepper_motor[MOTOR_NOZZLE].running_type = Motor_Running_Type_Positioning;
    stepper_motor[MOTOR_NOZZLE].target_position = nozzle_target;
    stepper_motor[MOTOR_NOZZLE].target_speed = speed;
    
    stepper_motor[MOTOR_BASE].running_type = Motor_Running_Type_Positioning;
    stepper_motor[MOTOR_BASE].target_position = base_target;
    stepper_motor[MOTOR_BASE].target_speed = speed;
}

bool stepper_motor_is_sync_complete(void)
{
    return (stepper_motor[MOTOR_NOZZLE].state == Motor_State_Stop &&
            stepper_motor[MOTOR_BASE].state == Motor_State_Stop &&
            stepper_motor[MOTOR_NOZZLE].position == stepper_motor[MOTOR_NOZZLE].target_position &&
            stepper_motor[MOTOR_BASE].position == stepper_motor[MOTOR_BASE].target_position);
}

bool stepper_motor_all_stopped(void)
{
    return (stepper_motor[MOTOR_NOZZLE].state == Motor_State_Stop &&
            stepper_motor[MOTOR_BASE].state == Motor_State_Stop);
}

// --- Configuration Functions ---
void stepper_motor_set_speed_profile(uint8_t index, uint16_t max_speed, uint8_t accel_rate)
{
    if(index < 2)
    {
        stepper_motor[index].target_speed = max_speed;
        stepper_motor[index].accelerate_rate = accel_rate;
    }
}
