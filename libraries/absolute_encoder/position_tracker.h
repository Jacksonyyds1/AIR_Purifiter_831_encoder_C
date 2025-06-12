#ifndef POSITION_TRACKER_H
#define POSITION_TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include "encoder_map.h"
#include "absolute_encoder_logger.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 位置跟踪器状态
 */
typedef enum
{
    TRACKER_STATE_UNINITIALIZED = 0,   // 未初始化
    TRACKER_STATE_TRACKING,            // 正常跟踪
    TRACKER_STATE_LOST,                // 跟踪丢失
    TRACKER_STATE_RECOVERING           // 恢复中
} tracker_state_t;

/**
 * @brief 跟踪验证结果
 */
typedef enum
{
    VALIDATION_RESULT_VALID = 0,       // 位置有效
    VALIDATION_RESULT_INVALID,         // 位置无效
    VALIDATION_RESULT_UNCERTAIN        // 不确定（需要更多数据）
} validation_result_t;

/**
 * @brief 位置跟踪器结构
 * 负责在已知位置基础上的实时位置跟踪
 */
typedef struct position_tracker_s
{
    encoder_map_t *encoder_map;         // 编码器映射引用
    const uint8_t *pattern;             // 码盘图案数据
    uint16_t pattern_length;            // 码盘图案长度
    
    // 跟踪状态
    tracker_state_t state;              // 当前状态
    uint16_t current_position;          // 当前位置
    int8_t movement_direction;          // 运动方向 (1=前进, -1=后退, 0=静止)
    uint8_t confidence_level;           // 置信度等级 (0-100)
    
    // 验证和恢复
    uint8_t *validation_buffer;         // 验证缓冲区
    uint8_t validation_buffer_size;     // 验证缓冲区大小
    uint8_t max_validation_length;      // 最大验证长度
    uint8_t validation_threshold;       // 验证阈值
    uint8_t recovery_attempts;          // 恢复尝试次数
    uint8_t max_recovery_attempts;      // 最大恢复尝试次数
    
    // 统计信息
    uint32_t total_updates;             // 总更新次数
    uint32_t valid_updates;             // 有效更新次数
    uint32_t lost_tracking_count;       // 跟踪丢失次数
    uint32_t recovery_success_count;    // 恢复成功次数
    
    // 内存管理
    bool use_external_buffer;           // 是否使用外部缓冲区
} position_tracker_t;

/**
 * @brief 创建位置跟踪器（动态内存版本）
 * @param encoder_map 编码器映射对象指针
 * @param pattern 码盘图案数据
 * @param pattern_length 码盘图案长度
 * @param max_validation_length 最大验证长度
 * @return 位置跟踪器指针，失败返回NULL
 */
position_tracker_t *position_tracker_create(encoder_map_t *encoder_map, const uint8_t *pattern,
                                            uint16_t pattern_length, uint8_t max_validation_length);

/**
 * @brief 创建位置跟踪器（静态缓冲区版本）
 * @param encoder_map 编码器映射对象指针
 * @param pattern 码盘图案数据
 * @param pattern_length 码盘图案长度
 * @param validation_buffer 预分配的验证缓冲区
 * @param max_validation_length 最大验证长度
 * @return 位置跟踪器指针，失败返回NULL
 */
position_tracker_t *position_tracker_create_with_buffer(encoder_map_t *encoder_map, const uint8_t *pattern,
                                                        uint16_t pattern_length, uint8_t *validation_buffer,
                                                        uint8_t max_validation_length);

/**
 * @brief 销毁位置跟踪器
 * @param tracker 位置跟踪器指针
 */
void position_tracker_destroy(position_tracker_t *tracker);

/**
 * @brief 设置初始位置，进入跟踪模式
 * @param tracker 位置跟踪器指针
 * @param position 初始位置
 * @param direction 初始运动方向
 * @return 是否成功
 */
bool position_tracker_set_initial_position(position_tracker_t *tracker, uint16_t position, int8_t direction);

/**
 * @brief 更新位置（基于新的编码器位值）
 * @param tracker 位置跟踪器指针
 * @param bit 新的编码器位值
 * @param direction 运动方向
 * @param new_position 输出参数：新位置
 * @param confidence 输出参数：置信度
 * @return 验证结果
 */
validation_result_t position_tracker_update_position(position_tracker_t *tracker, uint8_t bit, int8_t direction,
                                                     uint16_t *new_position, uint8_t *confidence);

/**
 * @brief 强制设置位置（用于纠错）
 * @param tracker 位置跟踪器指针
 * @param position 新位置
 * @param reset_confidence 是否重置置信度
 * @return 是否成功
 */
bool position_tracker_force_set_position(position_tracker_t *tracker, uint16_t position, bool reset_confidence);

/**
 * @brief 获取当前位置信息
 * @param tracker 位置跟踪器指针
 * @param position 输出参数：当前位置
 * @param confidence 输出参数：置信度
 * @param state 输出参数：状态
 * @return 是否成功
 */
bool position_tracker_get_current_info(const position_tracker_t *tracker, uint16_t *position, 
                                       uint8_t *confidence, tracker_state_t *state);

/**
 * @brief 重置跟踪器状态
 * @param tracker 位置跟踪器指针
 */
void position_tracker_reset(position_tracker_t *tracker);

/**
 * @brief 获取统计信息
 * @param tracker 位置跟踪器指针
 * @param total_updates 输出参数：总更新次数
 * @param valid_updates 输出参数：有效更新次数
 * @param lost_count 输出参数：跟踪丢失次数
 * @param recovery_count 输出参数：恢复成功次数
 */
void position_tracker_get_stats(const position_tracker_t *tracker, uint32_t *total_updates, 
                                uint32_t *valid_updates, uint32_t *lost_count, uint32_t *recovery_count);

/**
 * @brief 清零统计信息
 * @param tracker 位置跟踪器指针
 */
void position_tracker_clear_stats(position_tracker_t *tracker);

/**
 * @brief 设置验证参数
 * @param tracker 位置跟踪器指针
 * @param validation_threshold 验证阈值
 * @param max_recovery_attempts 最大恢复尝试次数
 */
void position_tracker_set_validation_params(position_tracker_t *tracker, uint8_t validation_threshold, 
                                            uint8_t max_recovery_attempts);

#ifdef __cplusplus
}
#endif

#endif // POSITION_TRACKER_H
