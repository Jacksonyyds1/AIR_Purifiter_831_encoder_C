#ifndef ABSOLUTE_ENCODER_H
#define ABSOLUTE_ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "absolute_encoder_logger.h"
#include "encoder_map.h"
#include "smart_search_manager.h"
#include "position_tracker.h"
#include "input_preprocessor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 编码器状态枚举
 */
typedef enum
{
    ENCODER_STATE_UNINITIALIZED = 0,   // 未初始化
    ENCODER_STATE_SEARCHING,           // 搜索模式
    ENCODER_STATE_TRACKING,            // 跟踪模式
    ENCODER_STATE_ERROR                // 错误状态
} encoder_state_t;

/**
 * @brief 编码器运行结果
 */
typedef enum
{
    ENCODER_RESULT_OK = 0,             // 正常
    ENCODER_RESULT_SEARCHING,          // 搜索中
    ENCODER_RESULT_POSITION_FOUND,     // 找到位置
    ENCODER_RESULT_TRACKING_UPDATED,   // 跟踪位置更新
    ENCODER_RESULT_ERROR_INVALID_SIGNAL, // 无效信号
    ENCODER_RESULT_ERROR_LOST_TRACKING,  // 跟踪丢失
    ENCODER_RESULT_ERROR_TIMEOUT       // 超时
} encoder_result_t;

/**
 * @brief 编码器配置参数
 */
typedef struct
{
    uint16_t motor_steps_per_unit;     // 电机多少步对应编码器前进1个单位
    uint8_t search_timeout_seconds;    // 搜索超时时间（秒）
    uint8_t tracking_lost_threshold;   // 跟踪丢失阈值
    uint32_t (*sys_tick_get)(void);    // 系统时钟获取函数指针
} encoder_config_t;

/**
 * @brief 编码器位置信息
 */
typedef struct
{
    uint32_t absolute_position;        // 绝对位置
    int32_t relative_position_change;  // 相对位置变化
    uint8_t confidence_level;          // 置信度等级 (0-100)
    encoder_state_t state;             // 当前状态
} encoder_position_t;

/**
 * @brief 编码器统计信息
 */
typedef struct
{
    uint32_t total_signals_processed;  // 处理的总信号数
    uint32_t search_attempts;          // 搜索尝试次数
    uint32_t tracking_updates;         // 跟踪更新次数
    uint32_t error_count;              // 错误计数
    uint32_t runtime_ticks;            // 运行时间（毫秒）
} encoder_stats_t;

// 不透明句柄类型
typedef struct encoder_handle_s *encoder_handle_t;
typedef struct encoder_map_handle_s *encoder_map_handle_t;

// 日志级别
extern int absolute_encoder_log_level;

/**
 * @brief 创建编码器图案对象
 * @param pattern_data 编码器图案数据
 * @param pattern_length 图案长度
 * @param max_search_depth 最大搜索深度
 * @return 编码器图案句柄，失败返回NULL
 */
encoder_map_handle_t create_encoder_map(const uint8_t *pattern_data, uint32_t pattern_length, uint8_t max_search_depth);

/**
 * @brief 创建编码器图案对象（使用内存池）
 * @param pattern_data 编码器图案数据
 * @param pattern_length 图案长度
 * @param max_search_depth 最大搜索深度
 * @param node_pool 内存池指针
 * @param pool_size 内存池大小
 * @return 编码器图案句柄，失败返回NULL
 */
encoder_map_handle_t create_encoder_map_with_pool(const uint8_t *pattern_data, uint32_t pattern_length, uint8_t max_search_depth, void *node_pool, uint16_t pool_size);

/**
 * @brief 销毁编码器图案对象
 * @param map_handle 编码器图案句柄
 */
void destroy_encoder_map(encoder_map_handle_t map_handle);

/**
 * @brief 创建编码器对象
 * @param map_handle 编码器图案句柄
 * @param config 编码器配置
 * @return 编码器句柄，失败返回NULL
 */
encoder_handle_t create_encoder(encoder_map_handle_t map_handle, const encoder_config_t *config);

/**
 * @brief 销毁编码器对象
 * @param handle 编码器句柄
 */
void destroy_encoder(encoder_handle_t handle);

/**
 * @brief 处理输入信号（主要接口）
 * @param handle 编码器句柄
 * @param signal_bit 输入信号位 (0或1)
 * @param position_info 输出位置信息
 * @return 处理结果
 */
encoder_result_t process_encoder_signal(encoder_handle_t handle, uint8_t signal_bit, encoder_position_t *position_info);

/**
 * @brief 处理电机步进信号
 * @param handle 编码器句柄
 * @param steps 步数
 * @param direction 方向 (1=前进, -1=后退)
 * @return 处理结果
 */
encoder_result_t process_motor_steps(encoder_handle_t handle, uint16_t steps, int8_t direction);

/**
 * @brief 获取当前位置信息
 * @param handle 编码器句柄
 * @param position_info 输出位置信息
 * @return 是否成功
 */
bool get_current_position(encoder_handle_t handle, encoder_position_t *position_info);

/**
 * @brief 获取编码器统计信息
 * @param handle 编码器句柄
 * @param stats 输出统计信息
 * @return 是否成功
 */
bool get_encoder_stats(encoder_handle_t handle, encoder_stats_t *stats);

/**
 * @brief 重置编码器状态
 * @param handle 编码器句柄
 * @return 是否成功
 */
bool reset_encoder(encoder_handle_t handle);

/**
 * @brief 强制切换到搜索模式
 * @param handle 编码器句柄
 * @return 是否成功
 */
bool force_search_mode(encoder_handle_t handle);

/**
 * @brief 设置日志级别
 * @param level 日志级别
 * @return 无
 */
void absolute_encoder_set_log_level(int level);

#ifdef __cplusplus
}
#endif

#endif // ABSOLUTE_ENCODER_H
