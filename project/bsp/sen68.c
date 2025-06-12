/**
  ******************************************************************************
  * @file    SEN68_simplified.c
  * @author  Generated Driver
  * @brief   SEN68 Environmental Sensor Driver (Simplified Non-blocking)
  *          This file provides firmware functions to manage the SEN68 sensor
  ******************************************************************************
  */

#include "SEN68.h"
#include "driver.h"
#include "ihastek.h"
#include <string.h>
#include "at32f403a_407_board.h"

/*
 * =============================================================================
 * SIMPLIFIED NON-BLOCKING DESIGN
 * =============================================================================
 * 
 * 1. Remove complex async state machine
 * 2. Keep non-blocking I2C operations
 * 3. Simple timing-based command execution
 * 4. Direct function calls with built-in timing
 * =============================================================================
 */

// Private global variables
static SEN68_Data_t g_sen68_data;
static SEN68_Control_t g_sen68_ctrl = {
    .state = SEN68_STATE_INITIALIZING,
    .measurement_running = false,
    .last_measurement_time = 0
};

// Simple command tracking
static struct {
    uint32_t send_time;     // When command was sent
    uint32_t wait_time;     // How long to wait
    bool pending;           // Command is pending
} g_command_state = {0, 0, false};

// Private function prototypes
static uint8_t SEN68_calculate_crc(const uint8_t *data, uint16_t count);
static uint8_t SEN68_send_command_nb(uint16_t command, uint8_t *data, uint16_t data_len);
static bool SEN68_can_read_response(uint32_t wait_ms);
static void SEN68_crc_init(void);
// ...existing function prototypes...

/**
  * @brief  Get current sensor data
  * @retval Pointer to SEN68_Data_t structure
  */
SEN68_Data_t *SEN68_Get_Data(void)
{
    return &g_sen68_data;
}

/**
  * @brief  Calculate CRC-8 checksum for SEN68 communication
  * @param  data: pointer to data buffer
  * @param  count: number of bytes to calculate CRC for
  * @retval CRC-8 checksum
  */
static uint8_t SEN68_calculate_crc(const uint8_t *data, uint16_t count)
{
    if(data == NULL || count == 0)
    {
        return 0xFF;
    }

    crc_data_reset();
    for(uint16_t i = 0; i < count; i++)
    {
        uint32_t word_data = (uint32_t)data[i] << 24;
        crc_one_word_calculate(word_data);
    }
    return (uint8_t)(crc_data_get() & 0xFF);
}

/**
  * @brief  Send command to SEN68 (non-blocking)
  * @param  command: 16-bit command ID
  * @param  data: pointer to data buffer
  * @param  data_len: data length in bytes
  * @retval SEN68_OK if success, SEN68_ERROR if failed
  */
static uint8_t SEN68_send_command_nb(uint16_t command, uint8_t *data, uint16_t data_len)
{
    uint8_t tx_buffer[32];
    uint16_t tx_len = 0;

    // Add command
    tx_buffer[tx_len++] = (command >> 8) & 0xFF;
    tx_buffer[tx_len++] = command & 0xFF;

    // Add data with CRC if provided
    if(data != NULL && data_len > 0)
    {
        for(uint16_t i = 0; i < data_len; i += 2)
        {
            tx_buffer[tx_len++] = data[i];
            if(i + 1 < data_len)
            {
                tx_buffer[tx_len++] = data[i + 1];
                tx_buffer[tx_len++] = SEN68_calculate_crc(&data[i], 2);
            }
            else
            {
                tx_buffer[tx_len++] = 0x00;
                uint8_t temp_data[2] = {data[i], 0x00};
                tx_buffer[tx_len++] = SEN68_calculate_crc(temp_data, 2);
            }
        }
    }

    // Send command and track timing
    if(drive_sen68_send_data(tx_buffer, tx_len) == SEN68_OK)
    {
        g_command_state.send_time = Get_Sys_Tick();
        g_command_state.pending = true;
        return SEN68_OK;
    }
    
    return SEN68_ERROR;
}

/**
  * @brief  Check if enough time has passed to read response
  * @param  wait_ms: minimum wait time in milliseconds
  * @retval true if can read, false if need to wait more
  */
static bool SEN68_can_read_response(uint32_t wait_ms)
{
    if(!g_command_state.pending)
    {
        return false;
    }
    
    return (Get_Sys_Tick() - g_command_state.send_time) >= wait_ms;
}

/**
  * @brief  Mark command as completed
  * @retval None
  */
static void SEN68_command_completed(void)
{
    g_command_state.pending = false;
}

/**
  * @brief  Initialize CRC hardware
  * @retval None
  */
static void SEN68_crc_init(void)
{
    crc_poly_size_set(CRC_POLY_SIZE_8B);
    crc_poly_value_set(0x31);
    crc_init_data_set(SEN68_CRC8_INIT);
    crc_reverse_input_data_set(CRC_REVERSE_INPUT_NO_AFFECTE);
    crc_reverse_output_data_set(CRC_REVERSE_OUTPUT_NO_AFFECTE);
}

/**
  * @brief  Parse measurement data from response buffer
  * @param  buffer: pointer to response buffer (27 bytes)
  * @retval true if successful, false if CRC error
  */
static bool SEN68_parse_measurement_data(uint8_t *buffer)
{
    // Verify CRC for each data word
    for(uint16_t i = 0; i < 27; i += 3)
    {
        if(SEN68_calculate_crc(&buffer[i], 2) != buffer[i + 2])
        {
            return false;
        }
    }

    // Parse data
    g_sen68_data.pm1_0 = ((uint16_t)buffer[0] << 8) | buffer[1];
    g_sen68_data.pm2_5 = ((uint16_t)buffer[3] << 8) | buffer[4];
    g_sen68_data.pm4_0 = ((uint16_t)buffer[6] << 8) | buffer[7];
    g_sen68_data.pm10_0 = ((uint16_t)buffer[9] << 8) | buffer[10];
    g_sen68_data.humidity = ((int16_t)buffer[12] << 8) | buffer[13];
    g_sen68_data.temperature = ((int16_t)buffer[15] << 8) | buffer[16];
    g_sen68_data.voc_index = ((int16_t)buffer[18] << 8) | buffer[19];
    g_sen68_data.nox_index = ((int16_t)buffer[21] << 8) | buffer[22];
    g_sen68_data.hcho_concentration = ((uint16_t)buffer[24] << 8) | buffer[25];

    // Handle invalid values
    if(g_sen68_data.pm1_0 == 0xFFFF) g_sen68_data.pm1_0 = 0;
    if(g_sen68_data.pm2_5 == 0xFFFF) g_sen68_data.pm2_5 = 0;
    if(g_sen68_data.pm4_0 == 0xFFFF) g_sen68_data.pm4_0 = 0;
    if(g_sen68_data.pm10_0 == 0xFFFF) g_sen68_data.pm10_0 = 0;
    if(g_sen68_data.humidity == 0x7FFF) g_sen68_data.humidity = 0;
    if(g_sen68_data.temperature == 0x7FFF) g_sen68_data.temperature = 0;
    if(g_sen68_data.voc_index == 0x7FFF) g_sen68_data.voc_index = 0;
    if(g_sen68_data.nox_index == 0x7FFF) g_sen68_data.nox_index = 0;
    if(g_sen68_data.hcho_concentration == 0xFFFF) g_sen68_data.hcho_concentration = 0;

    return true;
}

/**
  * @brief  Handle initializing state
  * @retval None
  */
static void SEN68_handle_initializing_state(void)
{
    static uint32_t init_start_time = 0;
    static bool command_sent = false;
    uint32_t tick = Get_Sys_Tick();
    
    if(init_start_time == 0)
    {
        init_start_time = tick;
        drive_sen68_power_up(true);
        command_sent = false;
    }

    // Wait for sensor startup
    if(tick - init_start_time > SEN68_STARTUP_TIME_MS)
    {
        if(!command_sent && !g_command_state.pending)
        {
            // Send product name command
            if(SEN68_send_command_nb(SEN68_CMD_GET_PRODUCT_NAME, NULL, 0) == SEN68_OK)
            {
                command_sent = true;
            }
        }
        else if(command_sent && SEN68_can_read_response(20))
        {
            uint8_t rx_buffer[48];
            if(drive_sen68_receive_data(rx_buffer, 48) == SEN68_OK)
            {
                g_sen68_ctrl.state = SEN68_STATE_IDLE;
                init_start_time = 0;
                command_sent = false;
            }
            else
            {
                // Retry after delay
                if(tick - init_start_time > 5000)
                {
                    init_start_time = 0;
                    command_sent = false;
                }
            }
            SEN68_command_completed();
        }
    }
}

/**
  * @brief  Handle idle state
  * @retval None
  */
static void SEN68_handle_idle_state(void)
{
    static bool start_command_sent = false;
    
    if(core_get_tek_power()->on && !g_sen68_ctrl.measurement_running)
    {
        if(!start_command_sent && !g_command_state.pending)
        {
            if(SEN68_send_command_nb(SEN68_CMD_START_MEASUREMENT, NULL, 0) == SEN68_OK)
            {
                start_command_sent = true;
            }
        }
        else if(start_command_sent && SEN68_can_read_response(50))
        {
            g_sen68_ctrl.state = SEN68_STATE_MEASUREMENT_STARTING;
            g_sen68_ctrl.measurement_running = true;
            g_sen68_ctrl.last_measurement_time = Get_Sys_Tick();
            start_command_sent = false;
            SEN68_command_completed();
        }
    }
}

/**
  * @brief  Handle measurement starting state
  * @retval None
  */
static void SEN68_handle_measurement_starting_state(void)
{
    static uint32_t stabilization_start_time = 0;
    uint32_t tick = Get_Sys_Tick();

    if(stabilization_start_time == 0)
    {
        stabilization_start_time = tick;
    }

    // Wait for sensor stabilization
    if(tick - stabilization_start_time > 1500)
    {
        g_sen68_ctrl.state = SEN68_STATE_MEASUREMENT_RUNNING;
        g_sen68_ctrl.last_measurement_time = tick;
        stabilization_start_time = 0;
    }
}

/**
  * @brief  Handle measurement running state
  * @retval None
  */
static void SEN68_handle_measurement_running_state(void)
{
    static enum {
        STEP_CHECK_READY,
        STEP_READ_DATA
    } measurement_step = STEP_CHECK_READY;
    
    uint32_t tick = Get_Sys_Tick();
    
    if(tick - g_sen68_ctrl.last_measurement_time >= SEN68_MEASUREMENT_INTERVAL_MS)
    {
        switch(measurement_step)
        {
        case STEP_CHECK_READY:
            if(!g_command_state.pending)
            {
                SEN68_send_command_nb(SEN68_CMD_GET_DATA_READY, NULL, 0);
            }
            else if(SEN68_can_read_response(20))
            {
                uint8_t rx_buffer[3];
                if(drive_sen68_receive_data(rx_buffer, 3) == SEN68_OK)
                {
                    if(SEN68_calculate_crc(rx_buffer, 2) == rx_buffer[2] && rx_buffer[1] == 1)
                    {
                        measurement_step = STEP_READ_DATA;
                    }
                }
                SEN68_command_completed();
            }
            break;
            
        case STEP_READ_DATA:
            if(!g_command_state.pending)
            {
                SEN68_send_command_nb(SEN68_CMD_READ_MEASURED_VALUES, NULL, 0);
            }
            else if(SEN68_can_read_response(20))
            {
                uint8_t data_buffer[27];
                if(drive_sen68_receive_data(data_buffer, 27) == SEN68_OK)
                {
                    SEN68_parse_measurement_data(data_buffer);
                }
                measurement_step = STEP_CHECK_READY;
                g_sen68_ctrl.last_measurement_time = tick;
                SEN68_command_completed();
            }
            break;
        }
    }
}

/**
  * @brief  Handle error state
  * @retval None
  */
static void SEN68_handle_error_state(void)
{
    static uint32_t error_recovery_time = 0;
    static bool reset_sent = false;
    uint32_t tick = Get_Sys_Tick();

    if(error_recovery_time == 0)
    {
        error_recovery_time = tick;
        g_sen68_ctrl.measurement_running = false;
        reset_sent = false;
    }

    if(tick - error_recovery_time > 5000)
    {
        if(!reset_sent && !g_command_state.pending)
        {
            if(SEN68_send_command_nb(SEN68_CMD_DEVICE_RESET, NULL, 0) == SEN68_OK)
            {
                reset_sent = true;
            }
        }
        else if(reset_sent && SEN68_can_read_response(1200))
        {
            g_sen68_ctrl.state = SEN68_STATE_INITIALIZING;
            error_recovery_time = 0;
            reset_sent = false;
            SEN68_command_completed();
        }
        else if(tick - error_recovery_time > 15000)
        {
            error_recovery_time = 0;
            reset_sent = false;
        }
    }
}

/**
  * @brief  Process SEN68 state machine
  * @retval None
  */
static void SEN68_state_process(void)
{
    switch(g_sen68_ctrl.state)
    {
    case SEN68_STATE_INITIALIZING:
        SEN68_handle_initializing_state();
        break;
    case SEN68_STATE_IDLE:
        SEN68_handle_idle_state();
        break;
    case SEN68_STATE_MEASUREMENT_STARTING:
        SEN68_handle_measurement_starting_state();
        break;
    case SEN68_STATE_MEASUREMENT_RUNNING:
        SEN68_handle_measurement_running_state();
        break;
    case SEN68_STATE_ERROR:
        SEN68_handle_error_state();
        break;
    default:
        g_sen68_ctrl.state = SEN68_STATE_INITIALIZING;
        break;
    }
}

/**
  * @brief  Initialize SEN68 sensor
  * @retval SEN68_OK if success, SEN68_ERROR if failed
  */
uint8_t SEN68_Init(void)
{
    SEN68_crc_init();
    g_sen68_ctrl.state = SEN68_STATE_INITIALIZING;
    g_sen68_ctrl.measurement_running = false;
    memset(&g_sen68_data, 0, sizeof(g_sen68_data));
    g_sen68_ctrl.last_measurement_time = Get_Sys_Tick();
    g_command_state.pending = false;

    return SEN68_OK;
}

/**
  * @brief  Start continuous measurement
  * @retval SEN68_OK if success, SEN68_ERROR if failed
  */
uint8_t SEN68_Start_Measurement(void)
{
    if(g_sen68_ctrl.measurement_running)
    {
        return SEN68_OK;
    }

    // Will be handled by state machine
    g_sen68_ctrl.measurement_running = true;
    g_sen68_ctrl.state = SEN68_STATE_MEASUREMENT_RUNNING;
    return SEN68_OK;
}

/**
  * @brief  Stop measurement (non-blocking)
  * @retval SEN68_OK if success, SEN68_ERROR if failed
  */
uint8_t SEN68_Stop_Measurement(void)
{
    if(!g_sen68_ctrl.measurement_running)
    {
        return SEN68_OK;
    }

    // Send stop command if no other command pending
    if(!g_command_state.pending)
    {
        if(SEN68_send_command_nb(SEN68_CMD_STOP_MEASUREMENT, NULL, 0) == SEN68_OK)
        {
            g_sen68_ctrl.measurement_running = false;
            g_sen68_ctrl.state = SEN68_STATE_IDLE;
            return SEN68_OK;
        }
    }
    
    return SEN68_ERROR;
}

/**
  * @brief  Check if measurement is running
  * @retval true if measurement running, false otherwise
  */
bool SEN68_Is_Measurement_Running(void)
{
    return g_sen68_ctrl.measurement_running;
}

// ...existing utility functions...

/**
  * @brief  SEN68 Handler function - call this periodically in main loop
  * @retval None
  */
void SEN68_Handler(void)
{
    static uint32_t last_tick = 0;

    if(Get_System_Tick() < 1)
    {
        return;
    }

    if(Get_Sys_Tick() - last_tick < 100)
    {
        return;
    }

    last_tick = Get_Sys_Tick();

    if(core_get_tek_power()->on)
    {
        SEN68_state_process();
    }
    else
    {
        if(g_sen68_ctrl.measurement_running)
        {
            SEN68_Stop_Measurement();
        }
        
        drive_sen68_power_up(false);
        g_sen68_ctrl.state = SEN68_STATE_IDLE;
        g_command_state.pending = false;
        memset(&g_sen68_data, 0, sizeof(g_sen68_data));
    }
}