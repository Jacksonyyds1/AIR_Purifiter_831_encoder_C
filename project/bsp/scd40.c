#include "scd40.h"
#include "ihastek.h"
#include "driver.h"

// Static variables
static scd4x_data_t scd4x_data = {0};
static uint8_t rx_buffer[32];
static uint8_t rx_index = 0;
static bool frame_receiving = false;
static uint8_t frame_complete = 0;

// Private function declarations
static bool SCD40_SendFrame(uint8_t cmd, uint8_t *data, uint8_t len);
static bool SCD40_ReceiveFrame(uint8_t *data, uint8_t *len);
static uint8_t SCD40_CalculateChecksum(uint8_t *data, uint8_t len);
static void SCD40_ByteStuffing(uint8_t *src, uint8_t src_len, uint8_t *dst, uint8_t *dst_len);
static void SCD40_ByteUnstuffing(uint8_t *src, uint8_t src_len, uint8_t *dst, uint8_t *dst_len);

void SCD40_Init(void)
{
    scd4x_data.state = SCD4X_STATE_INITIALIZING;
    scd4x_data.data_ready = false;
    scd4x_data.error_code = SCD4X_ERROR_NO_ERROR;
    scd4x_data.co2_ppm = 0;

    // 开启CO2传感器电源
    driver_set_SCD40_Power(1);

    // 等待初始化完成（1100ms）
    scd4x_data.last_measurement_time = Get_Sys_Tick();
}

void SCD40_Handler(void)
{
    static unsigned long last_tick = 0;
    static uint32_t init_start_time = 0;

    if(Get_System_Tick() < 1)
    {
        return;
    }

    if(Get_Sys_Tick() - last_tick < 100) // 100ms处理间隔
    {
        return;
    }

    last_tick = Get_Sys_Tick();

    if(core_get_tek_power()->on)
    {
        switch(scd4x_data.state)
        {
        case SCD4X_STATE_INITIALIZING:
            if(init_start_time == 0)
            {
                init_start_time = Get_Sys_Tick();
            }
            // 等待1100ms初始化完成
            if(Get_Sys_Tick() - init_start_time > 1100)
            {
                scd4x_data.state = SCD4X_STATE_IDLE;
                init_start_time = 0;
            }
            break;

        case SCD4X_STATE_IDLE:
            // 开始测量
            if(SCD40_Start_Measurement())
            {
                scd4x_data.state = SCD4X_STATE_MEASUREMENT_STARTING;
                scd4x_data.last_measurement_time = Get_Sys_Tick();
            }
            break;

        case SCD4X_STATE_MEASUREMENT_STARTING:
            // 等待6秒后第一个测量值
            if(Get_Sys_Tick() - scd4x_data.last_measurement_time > 6000)
            {
                scd4x_data.state = SCD4X_STATE_MEASUREMENT_RUNNING;
            }
            break;

        case SCD4X_STATE_MEASUREMENT_RUNNING:
            // 每秒读取一次数据
            if(Get_Sys_Tick() - scd4x_data.last_measurement_time > 1000)
            {
                uint16_t co2_value;
                if(SCD40_Read_Measurement(&co2_value))
                {
                    scd4x_data.co2_ppm = co2_value;
                    scd4x_data.data_ready = true;
                }
                scd4x_data.last_measurement_time = Get_Sys_Tick();
            }
            break;

        case SCD4X_STATE_ERROR:
            // 尝试复位传感器
            if(SCD40_Reset())
            {
                scd4x_data.state = SCD4X_STATE_INITIALIZING;
                init_start_time = Get_Sys_Tick();
            }
            break;

        default:
            break;
        }
    }
    else
    {
        drive_set_scd40_power_up(false); // 关闭CO2传感器电源
        scd4x_data.state = SCD4X_STATE_IDLE;
        scd4x_data.data_ready = false;
    }
}

bool SCD40_Start_Measurement(void)
{
    uint8_t cmd_data = 0x00; // 子命令
    return SCD40_SendFrame(SCD4X_CMD_START_MEASUREMENT, &cmd_data, 1);
}

bool SCD40_Stop_Measurement(void)
{
    return SCD40_SendFrame(SCD4X_CMD_STOP_MEASUREMENT, NULL, 0);
}

bool SCD40_Read_Measurement(uint16_t *co2_ppm)
{
    uint8_t cmd_data = 0x00; // 子命令
    uint8_t rx_data[4];
    uint8_t rx_len;

    if(!SCD40_SendFrame(SCD4X_CMD_READ_MEASUREMENT, &cmd_data, 1))
    {
        return false;
    }

    if(!SCD40_ReceiveFrame(rx_data, &rx_len))
    {
        return false;
    }

    if(rx_len >= 2)
    {
        *co2_ppm = (rx_data[0] << 8) | rx_data[1];
        return true;
    }

    return false;
}

bool SCD40_Self_Test(void)
{
    uint8_t cmd_data = 0x00; // 开始自检
    return SCD40_SendFrame(SCD4X_CMD_SELF_TEST, &cmd_data, 1);
}

bool SCD40_Reset(void)
{
    return SCD40_SendFrame(SCD4X_CMD_RESET, NULL, 0);
}

scd4x_data_t *SCD40_Get_Data(void)
{
    return &scd4x_data;
}

// Private functions
static bool SCD40_SendFrame(uint8_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t frame[64];
    uint8_t frame_len = 0;
    uint8_t checksum;

    // 构建帧
    frame[frame_len++] = SCD4X_FRAME_START_STOP; // 开始字节
    frame[frame_len++] = SCD4X_SLAVE_ADDR;       // 地址
    frame[frame_len++] = cmd;                    // 命令
    frame[frame_len++] = len;                    // 长度

    // 添加数据
    if(data && len > 0)
    {
        for(uint8_t i = 0; i < len; i++)
        {
            frame[frame_len++] = data[i];
        }
    }

    // 计算校验和（不包括开始和结束字节）
    checksum = SCD40_CalculateChecksum(&frame[1], frame_len - 1);
    frame[frame_len++] = checksum;

    // 字节填充处理
    uint8_t stuffed_frame[128];
    uint8_t stuffed_len;
    SCD40_ByteStuffing(&frame[1], frame_len - 1, stuffed_frame, &stuffed_len);

    // 发送帧
    driver_SCD40_UART_Send(SCD4X_FRAME_START_STOP);
    for(uint8_t i = 0; i < stuffed_len; i++)
    {
        driver_SCD40_UART_Send(stuffed_frame[i]);
    }
    driver_SCD40_UART_Send(SCD4X_FRAME_START_STOP);

    return true;
}

static bool SCD40_ReceiveFrame(uint8_t *data, uint8_t *len)
{
    static uint32_t rx_timeout = 0;

    // 设置超时时间为50ms
    if(rx_timeout == 0)
    {
        rx_timeout = Get_Sys_Tick() + 50;
    }

    // 检查是否接收到完整帧
    if(frame_complete)
    {
        // 处理接收到的数据
        if(rx_index >= 5) // 至少包含地址、命令、状态、长度、校验和
        {
            uint8_t unstuffed_data[32];
            uint8_t unstuffed_len;

            // 字节去填充
            SCD40_ByteUnstuffing(rx_buffer, rx_index, unstuffed_data, &unstuffed_len);

            // 验证校验和
            uint8_t calc_checksum = SCD40_CalculateChecksum(unstuffed_data, unstuffed_len - 1);
            if(calc_checksum == unstuffed_data[unstuffed_len - 1])
            {
                // 提取数据部分（跳过地址、命令、状态、长度）
                uint8_t data_len = unstuffed_data[3];
                if(data_len > 0 && unstuffed_len >= (4 + data_len + 1))
                {
                    for(uint8_t i = 0; i < data_len; i++)
                    {
                        data[i] = unstuffed_data[4 + i];
                    }
                    *len = data_len;
                }
                else
                {
                    *len = 0;
                }

                frame_complete = 0;
                rx_timeout = 0;
                return true;
            }
        }

        frame_complete = 0;
        rx_timeout = 0;
        return false;
    }

    // 检查超时
    if(Get_Sys_Tick() > rx_timeout)
    {
        frame_complete = 0;
        rx_timeout = 0;
        return false;
    }

    return false; // 还未接收完成
}

static uint8_t SCD40_CalculateChecksum(uint8_t *data, uint8_t len)
{
    uint16_t sum = 0;
    for(uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return ~(sum & 0xFF); // 取LSB并取反
}

static void SCD40_ByteStuffing(uint8_t *src, uint8_t src_len, uint8_t *dst, uint8_t *dst_len)
{
    uint8_t j = 0;

    for(uint8_t i = 0; i < src_len; i++)
    {
        switch(src[i])
        {
        case 0x7E:
            dst[j++] = 0x7D;
            dst[j++] = 0x5E;
            break;
        case 0x7D:
            dst[j++] = 0x7D;
            dst[j++] = 0x5D;
            break;
        case 0x11:
            dst[j++] = 0x7D;
            dst[j++] = 0x31;
            break;
        case 0x13:
            dst[j++] = 0x7D;
            dst[j++] = 0x33;
            break;
        default:
            dst[j++] = src[i];
            break;
        }
    }

    *dst_len = j;
}

static void SCD40_ByteUnstuffing(uint8_t *src, uint8_t src_len, uint8_t *dst, uint8_t *dst_len)
{
    uint8_t j = 0;

    for(uint8_t i = 0; i < src_len; i++)
    {
        if(src[i] == 0x7D && i + 1 < src_len)
        {
            switch(src[i + 1])
            {
            case 0x5E:
                dst[j++] = 0x7E;
                i++; // 跳过下一个字节
                break;
            case 0x5D:
                dst[j++] = 0x7D;
                i++; // 跳过下一个字节
                break;
            case 0x31:
                dst[j++] = 0x11;
                i++; // 跳过下一个字节
                break;
            case 0x33:
                dst[j++] = 0x13;
                i++; // 跳过下一个字节
                break;
            default:
                dst[j++] = src[i];
                break;
            }
        }
        else
        {
            dst[j++] = src[i];
        }
    }

    *dst_len = j;
}

void driver_SCD40_UART_Receive_Handler(uint8_t received_byte)
{
    // UART接收中断处理函数，需要在UART中断中调用
    // 这里实现SCD40传感器的数据接收逻辑
    extern uint8_t rx_buffer[32];
    extern uint8_t rx_index;
    extern bool frame_receiving;
    extern uint8_t frame_complete;

    static bool escape_next = false;

    if(received_byte == 0x7E)
    {
        if(frame_receiving)
        {
            // 帧结束
            frame_receiving = false;
            frame_complete = 1; // 设置帧完成标志
        }
        else
        {
            // 帧开始
            frame_receiving = true;
            rx_index = 0;
            escape_next = false;
            frame_complete = 0; // 重置帧完成标志
        }
    }
    else if(frame_receiving)
    {
        if(escape_next)
        {
            // 处理转义字节
            switch(received_byte)
            {
            case 0x5E:
                if(rx_index < sizeof(rx_buffer))
                {
                    rx_buffer[rx_index++] = 0x7E;
                }
                break;
            case 0x5D:
                if(rx_index < sizeof(rx_buffer))
                {
                    rx_buffer[rx_index++] = 0x7D;
                }
                break;
            case 0x31:
                if(rx_index < sizeof(rx_buffer))
                {
                    rx_buffer[rx_index++] = 0x11;
                }
                break;
            case 0x33:
                if(rx_index < sizeof(rx_buffer))
                {
                    rx_buffer[rx_index++] = 0x13;
                }
                break;
            default:
                if(rx_index < sizeof(rx_buffer))
                {
                    rx_buffer[rx_index++] = received_byte;
                }
                break;
            }
            escape_next = false;
        }
        else if(received_byte == 0x7D)
        {
            // 转义字符
            escape_next = true;
        }
        else
        {
            // 正常数据
            if(rx_index < sizeof(rx_buffer))
            {
                rx_buffer[rx_index++] = received_byte;
            }
        }

        // 防止缓冲区溢出
        if(rx_index >= sizeof(rx_buffer))
        {
            frame_receiving = false;
            rx_index = 0;
            frame_complete = 0;
        }
    }
}
