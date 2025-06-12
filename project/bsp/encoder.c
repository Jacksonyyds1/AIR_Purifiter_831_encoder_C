#include "at32f403a_407_board.h"
#include "absolute_encoder.h"
#include "encoder.h"
#include "stepper_motor.h"
#include "define.h"

const uint8_t encoder_map_data[360] = { // k=9的德布鲁因序列，裁剪至360位，从任意位置出发最长可在10位内定位
    1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,0,1,0,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0,1,1,0,1,1,1,1,
    1,0,1,0,0,1,1,1,1,1,0,0,1,0,1,1,1,1,1,0,0,0,0,1,1,1,1,0,1,1,1,0,1,1,1,1,0,1,1,0,0,1,1,1,1,0,1,0,1,0,
    1,1,1,1,0,1,0,0,0,1,1,1,1,0,0,1,1,0,1,1,1,1,0,0,1,0,0,1,1,1,1,0,0,0,1,0,1,1,1,1,0,0,0,0,0,1,1,1,0,1,
    1,1,0,0,1,1,1,0,1,1,0,1,0,1,1,1,0,1,1,0,0,0,1,1,1,0,1,0,1,1,0,1,1,1,0,1,0,1,0,0,1,1,1,0,1,0,0,1,0,1,
    1,1,0,1,0,0,0,0,1,1,1,0,0,1,1,0,0,1,1,1,0,0,1,0,1,0,1,1,1,0,0,1,0,0,0,1,1,1,0,0,0,1,1,0,1,1,1,0,0,0,
    1,0,0,1,1,1,0,0,0,0,1,0,1,1,1,0,0,0,0,0,0,1,1,0,1,1,0,1,1,0,1,0,0,1,1,0,1,1,0,0,1,0,1,1,0,1,1,0,0,0,
    0,1,1,0,1,0,1,1,0,0,1,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,0,0,1,1,0,1,0,0,1,0,0,1,1,0,1,0,0,0,1,0,1,1,0,0,
    1,0,0,0,0,0,0,0,0,0
};

uint8_t encoder_map_node_pool[40 * 768] = {0};

encoder_map_handle_t map_handle;
encoder_handle_t encoder_nozzle_handle, encoder_base_handle;

void encoder_init(void)
{
    encoder_config_t encoder_config;
    encoder_config.search_timeout_seconds = 0;  // 搜索超时时间（秒），0表示不超时
    encoder_config.tracking_lost_threshold = 3; // 连续3次失败切换搜索模式
    encoder_config.sys_tick_get = Get_Sys_Tick; // 系统时钟获取函数指针
    //map_handle = create_encoder_map_with_pool(encoder_map_data, 360, 12, encoder_map_node_pool, 40 * 768);
    map_handle = create_encoder_map_with_pool(encoder_map_data, 360, 12, encoder_map_node_pool, 768);

    encoder_config.motor_steps_per_unit = 100;  // TODO: 电机多少步对应编码器前进1个单位，需根据实际情况进行修改
    encoder_nozzle_handle = create_encoder(map_handle, &encoder_config);
    encoder_config.motor_steps_per_unit = 100;  // TODO: 电机多少步对应编码器前进1个单位，需根据实际情况进行修改
    encoder_base_handle = create_encoder(map_handle, &encoder_config);
}

void encoder_input_position(uint8_t index, int direction, uint16_t steps)
{
    int encoder_sig;
    if(index == MOTOR_NOZZLE)
    {
        encoder_sig = gpio_input_data_bit_read(IR_GATE_NOZZLE_PORT, IR_GATE_NOZZLE_PIN);
        if(encoder_nozzle_handle)
        {
            process_motor_steps(encoder_nozzle_handle, steps, (direction == (int)Motor_Direction_Forward) ? 1 : -1);
        }
    }
    else if(index == MOTOR_BASE)
    {
        encoder_sig = gpio_input_data_bit_read(IR_GATE_BASE_PORT, IR_GATE_BASE_PIN);
        if(encoder_base_handle)
        {
            process_motor_steps(encoder_base_handle, steps, (direction == (int)Motor_Direction_Forward) ? 1 : -1);
        }
    }
}
