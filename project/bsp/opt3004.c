/**
  ******************************************************************************
  * @file    opt3004.c
  * @author  Generated Driver
  * @brief   OPT3004 Light Sensor Driver (Simplified)
  *          This file provides firmware functions to manage the OPT3004 sensor
  ******************************************************************************
  */

#include "opt3004.h"
#include "driver.h"
#include "ihastek.h"
#include <string.h>
#include "at32f403a_407_board.h"

/*
 * =============================================================================
 * GLOBAL VARIABLES ORGANIZATION
 * =============================================================================
 *
 * 1. Sensor Data Storage:
 *    - g_opt3004_data: Latest measurement values from sensor
 *
 * 2. Sensor Control:
 *    - g_opt3004_ctrl: State machine and measurement control
 * =============================================================================
 */

// Private global variables - organized by functionality

// Sensor data storage
static OPT3004_Data_t g_opt3004_data;

// Sensor control
static OPT3004_Control_t g_opt3004_ctrl =
{
    .state = OPT3004_STATE_INITIALIZING,
    .measurement_running = false,
    .last_measurement_time = 0,
    .current_step = OPT3004_STEP_CHECK_CRF
};

/**
  * @brief  Get pointer to sensor data
  * @retval Pointer to OPT3004_Data_t structure containing sensor data
  */
OPT3004_Data_t *OPT3004_Get_Data(void)
{
    return &g_opt3004_data;
}

/**
  * @brief  Read register from OPT3004 sensor
  * @param  reg_addr: register address
  * @param  data: pointer to store read data
  * @retval OPT3004_OK if success, OPT3004_ERROR if failed
  */
static uint8_t OPT3004_read_register(uint8_t reg_addr, uint16_t *data)
{
    uint8_t result;
    uint8_t rx_buffer[2];

    // Send register address
    result = drive_opt3004_send_data(&reg_addr, 1);
    if(result != OPT3004_OK)
    {
        return OPT3004_ERROR;
    }

    // Small delay for register access
    delay_ms(2);

    // Read 2 bytes (register data is 16-bit, MSB first)
    result = drive_opt3004_receive_data(rx_buffer, 2);
    if(result != OPT3004_OK)
    {
        return OPT3004_ERROR;
    }

    // Combine MSB and LSB
    *data = ((uint16_t)rx_buffer[0] << 8) | rx_buffer[1];

    return OPT3004_OK;
}

/**
  * @brief  Write register to OPT3004 sensor
  * @param  reg_addr: register address
  * @param  data: data to write
  * @retval OPT3004_OK if success, OPT3004_ERROR if failed
  */
static uint8_t OPT3004_write_register(uint8_t reg_addr, uint16_t data)
{
    uint8_t tx_buffer[3];

    tx_buffer[0] = reg_addr;
    tx_buffer[1] = (data >> 8) & 0xFF;  // MSB
    tx_buffer[2] = data & 0xFF;         // LSB

    return drive_opt3004_send_data(tx_buffer, 3);
}

/**
  * @brief  Calculate lux value from raw sensor data
  * @param  raw_data: raw 16-bit sensor data
  * @retval calculated lux value
  */
static float OPT3004_calculate_lux(uint16_t raw_data)
{
    uint16_t exponent = (raw_data >> 12) & 0x0F;
    uint16_t result = raw_data & 0x0FFF;

    // Formula: lux = 0.01 * 2^exponent * result
    float lux = 0.01f * (1 << exponent) * result;

    return lux;
}

/**
  * @brief  Check if conversion is ready (CRF flag)
  * @param  config_reg: configuration register value
  * @retval true if conversion ready, false otherwise
  */
static bool OPT3004_check_conversion_ready(uint16_t config_reg)
{
    return (config_reg & OPT3004_CONFIG_CRF) != 0;
}

/**
  * @brief  Handle initializing state
  * @retval None
  */
static void OPT3004_handle_initializing_state(void)
{
    static uint32_t init_start_time = 0;
    uint32_t tick = Get_Sys_Tick();

    if(init_start_time == 0)
    {
        init_start_time = tick;
    }

    // 等待传感器启动 (100ms)
    if(tick - init_start_time > OPT3004_STARTUP_TIME_MS)
    {
        uint16_t device_id;
        if(OPT3004_read_register(OPT3004_REG_DEVICE_ID, &device_id) == OPT3004_OK)
        {
            if(device_id == OPT3004_DEVICE_ID)
            {
                g_opt3004_ctrl.state = OPT3004_STATE_CONFIGURING;
                init_start_time = 0;
            }
            else
            {
                g_opt3004_ctrl.state = OPT3004_STATE_ERROR;
            }
        }
        else
        {
            // 初始化失败，延迟后重试
            if(tick - init_start_time > 5000) // 每5秒重试一次
            {
                init_start_time = 0;
            }
        }
    }
}

/**
  * @brief  Handle configuring state
  * @retval None
  */
static void OPT3004_handle_configuring_state(void)
{
    // Configure: Auto range, 800ms conversion time, continuous mode
    uint16_t config = OPT3004_CONFIG_RN_AUTO | OPT3004_CONFIG_CT_800MS |
                      OPT3004_CONFIG_M_CONTINUOUS | OPT3004_CONFIG_FC_1;

    if(OPT3004_write_register(OPT3004_REG_CONFIG, config) == OPT3004_OK)
    {
        g_opt3004_ctrl.state = OPT3004_STATE_IDLE;
    }
    else
    {
        g_opt3004_ctrl.state = OPT3004_STATE_ERROR;
    }
}

/**
  * @brief  Handle idle state
  * @retval None
  */
static void OPT3004_handle_idle_state(void)
{
    // 检查是否需要重新启动测量
    if(core_get_tek_power()->on && !g_opt3004_ctrl.measurement_running)
    {
        // 重新配置为连续模式
        uint16_t config = OPT3004_CONFIG_RN_AUTO | OPT3004_CONFIG_CT_800MS |
                          OPT3004_CONFIG_M_CONTINUOUS | OPT3004_CONFIG_FC_1;

        if(OPT3004_write_register(OPT3004_REG_CONFIG, config) == OPT3004_OK)
        {
            g_opt3004_ctrl.state = OPT3004_STATE_MEASUREMENT_RUNNING;
            g_opt3004_ctrl.measurement_running = true;
            g_opt3004_ctrl.last_measurement_time = Get_Sys_Tick();
            g_opt3004_ctrl.current_step = OPT3004_STEP_CHECK_CRF;
        }
        else
        {
            g_opt3004_ctrl.state = OPT3004_STATE_ERROR;
        }
    }
}

/**
  * @brief  Handle measurement running state with CRF checking
  * @retval None
  */
static void OPT3004_handle_measurement_running_state(void)
{
    uint32_t tick = Get_Sys_Tick();

    // 检查是否到了读取间隔
    if(tick - g_opt3004_ctrl.last_measurement_time >= 100) // 100ms检查间隔
    {
        switch(g_opt3004_ctrl.current_step)
        {
        case OPT3004_STEP_CHECK_CRF:
        {
            uint16_t config_reg;
            if(OPT3004_read_register(OPT3004_REG_CONFIG, &config_reg) == OPT3004_OK)
            {
                if(OPT3004_check_conversion_ready(config_reg))
                {
                    // 有新数据可用，切换到读取数据步骤
                    g_opt3004_ctrl.current_step = OPT3004_STEP_READ_DATA;

                    // 检查溢出标志
                    g_opt3004_data.overflow = (config_reg & OPT3004_CONFIG_OVF) != 0;
                }
                // 如果CRF=0，继续等待下次检查
            }
            else
            {
                g_opt3004_ctrl.state = OPT3004_STATE_ERROR;
            }
        }
        break;

        case OPT3004_STEP_READ_DATA:
        {
            uint16_t result_data;
            if(OPT3004_read_register(OPT3004_REG_RESULT, &result_data) == OPT3004_OK)
            {
                // 解析并存储数据
                g_opt3004_data.result_reg = result_data;
                g_opt3004_data.raw_lux = result_data & 0x0FFF;
                g_opt3004_data.exponent = (result_data >> 12) & 0x0F;
                g_opt3004_data.lux = OPT3004_calculate_lux(result_data);
                g_opt3004_data.conversion_ready = true;

                // 读取完成后，CRF会自动清0，回到检查CRF步骤
                g_opt3004_ctrl.current_step = OPT3004_STEP_CHECK_CRF;
            }
            else
            {
                g_opt3004_ctrl.state = OPT3004_STATE_ERROR;
            }
        }
        break;
        }

        g_opt3004_ctrl.last_measurement_time = tick;
    }
}

/**
  * @brief  Handle error state
  * @retval None
  */
static void OPT3004_handle_error_state(void)
{
    static uint32_t error_recovery_time = 0;
    uint32_t tick = Get_Sys_Tick();

    if(error_recovery_time == 0)
    {
        error_recovery_time = tick;
        g_opt3004_ctrl.measurement_running = false;
    }

    // 等待5秒后尝试恢复
    if(tick - error_recovery_time > 5000)
    {
        // 重新初始化
        g_opt3004_ctrl.state = OPT3004_STATE_INITIALIZING;
        error_recovery_time = 0;
    }
}

/**
  * @brief  State machine processor
  * @retval None
  */
static void OPT3004_state_process(void)
{
    switch(g_opt3004_ctrl.state)
    {
    case OPT3004_STATE_INITIALIZING:
        OPT3004_handle_initializing_state();
        break;
    case OPT3004_STATE_CONFIGURING:
        OPT3004_handle_configuring_state();
        break;
    case OPT3004_STATE_IDLE:
        OPT3004_handle_idle_state();
        break;
    case OPT3004_STATE_MEASUREMENT_RUNNING:
        OPT3004_handle_measurement_running_state();
        break;
    case OPT3004_STATE_ERROR:
        OPT3004_handle_error_state();
        break;
    default:
        g_opt3004_ctrl.state = OPT3004_STATE_INITIALIZING;
        break;
    }
}

/**
  * @brief  Initialize OPT3004 sensor
  * @retval OPT3004_OK if success, OPT3004_ERROR if failed
  */
uint8_t OPT3004_Init(void)
{
    // 初始化状态
    g_opt3004_ctrl.state = OPT3004_STATE_INITIALIZING;
    g_opt3004_ctrl.measurement_running = false;
    g_opt3004_ctrl.current_step = OPT3004_STEP_CHECK_CRF;
    memset(&g_opt3004_data, 0, sizeof(g_opt3004_data));
    g_opt3004_ctrl.last_measurement_time = Get_Sys_Tick();

    return OPT3004_OK;
}

/**
  * @brief  Start continuous measurement
  * @retval OPT3004_OK if success, OPT3004_ERROR if failed
  */
uint8_t OPT3004_Start_Measurement(void)
{
    if(g_opt3004_ctrl.measurement_running)
    {
        return OPT3004_OK; // 已经在运行，返回成功
    }

    // 重新配置为连续模式
    uint16_t config = OPT3004_CONFIG_RN_AUTO | OPT3004_CONFIG_CT_800MS |
                      OPT3004_CONFIG_M_CONTINUOUS | OPT3004_CONFIG_FC_1;

    if(OPT3004_write_register(OPT3004_REG_CONFIG, config) == OPT3004_OK)
    {
        g_opt3004_ctrl.measurement_running = true;
        g_opt3004_ctrl.state = OPT3004_STATE_MEASUREMENT_RUNNING;
        g_opt3004_ctrl.current_step = OPT3004_STEP_CHECK_CRF;
        g_opt3004_ctrl.last_measurement_time = Get_Sys_Tick();
        return OPT3004_OK;
    }

    return OPT3004_ERROR;
}

/**
  * @brief  Stop measurement and enter shutdown mode
  * @retval OPT3004_OK if success, OPT3004_ERROR if failed
  */
uint8_t OPT3004_Stop_Measurement(void)
{
    if(!g_opt3004_ctrl.measurement_running)
    {
        return OPT3004_ERROR;
    }

    // 设置传感器为shutdown模式以节省功耗
    uint16_t config = OPT3004_CONFIG_RN_AUTO | OPT3004_CONFIG_CT_800MS |
                      OPT3004_CONFIG_M_SHUTDOWN | OPT3004_CONFIG_FC_1;

    if(OPT3004_write_register(OPT3004_REG_CONFIG, config) == OPT3004_OK)
    {
        g_opt3004_ctrl.measurement_running = false;
        g_opt3004_ctrl.state = OPT3004_STATE_IDLE;
        return OPT3004_OK;
    }

    return OPT3004_ERROR;
}

/**
  * @brief  Get lux value
  * @retval current lux value
  */
float OPT3004_Get_Lux(void)
{
    return g_opt3004_data.lux;
}

/**
  * @brief  Check if measurement is running
  * @retval true if measurement running, false otherwise
  */
bool OPT3004_Is_Measurement_Running(void)
{
    return g_opt3004_ctrl.measurement_running;
}

/**
  * @brief  Set sensor configuration
  * @param  config: configuration value
  * @retval OPT3004_OK if success, OPT3004_ERROR if failed
  */
uint8_t OPT3004_Set_Config(uint16_t config)
{
    return OPT3004_write_register(OPT3004_REG_CONFIG, config);
}

/**
  * @brief  Get sensor configuration
  * @param  config: pointer to store configuration value
  * @retval OPT3004_OK if success, OPT3004_ERROR if failed
  */
uint8_t OPT3004_Get_Config(uint16_t *config)
{
    return OPT3004_read_register(OPT3004_REG_CONFIG, config);
}

/**
  * @brief  Check device ID
  * @retval OPT3004_OK if device ID matches, OPT3004_ERROR if not
  */
uint8_t OPT3004_Check_Device_ID(void)
{
    uint16_t device_id;
    uint8_t result = OPT3004_read_register(OPT3004_REG_DEVICE_ID, &device_id);

    if(result == OPT3004_OK && device_id == OPT3004_DEVICE_ID)
    {
        return OPT3004_OK;
    }

    return OPT3004_ERROR;
}

/**
  * @brief  OPT3004 Handler function - call this periodically in main loop
  * @retval None
  */
void OPT3004_Handler(void)
{
    static uint32_t last_tick = 0;

    if(Get_System_Tick() < 1)
    {
        return;
    }

    // 100ms处理间隔
    if(Get_Sys_Tick() - last_tick < 100)
    {
        return;
    }

    last_tick = Get_Sys_Tick();

    // 检查传感器是否应该开启
    if(core_get_tek_power()->on)
    {
        OPT3004_state_process();
    }
    else
    {
        // 系统电源关闭，主循环可直接调用停止测量接口
        if(g_opt3004_ctrl.measurement_running)
        {
            OPT3004_Stop_Measurement(); // 直接调用停止测量接口
        }

        // 重置传感器状态
        g_opt3004_ctrl.state = OPT3004_STATE_IDLE;
        g_opt3004_ctrl.current_step = OPT3004_STEP_CHECK_CRF;
        memset(&g_opt3004_data, 0, sizeof(g_opt3004_data));
    }
}