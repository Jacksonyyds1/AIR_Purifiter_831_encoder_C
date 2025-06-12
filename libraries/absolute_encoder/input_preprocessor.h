#ifndef INPUT_PREPROCESSOR_H
#define INPUT_PREPROCESSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "absolute_encoder_logger.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 换算配置参数
 */
typedef struct
{
    uint16_t motor_steps_per_unit;      // 电机多少步对应编码器前进1个单位
    float step_tolerance_ratio;         // 步数容差比例 (0.0-1.0)
    uint16_t max_step_deviation;        // 最大允许步数偏差
    bool enable_adaptive_correction;    // 是否启用自适应修正
    bool enable_auto_alignment;         // 是否启用自动对齐
    uint16_t alignment_sample_count;    // 对齐采样数量
    float alignment_tolerance;          // 对齐容差
} conversion_config_t;

/**
 * @brief 滤波配置参数
 */
typedef struct
{
    uint8_t debounce_samples;           // 防抖采样数
    bool enable_direction_filter;       // 是否启用方向滤波
    uint8_t direction_filter_window;    // 方向滤波窗口大小
    uint8_t consistency_threshold;      // 连续相同信号阈值
    bool enable_pattern_validation;     // 是否启用模式验证
    bool enable_auto_alignment;         // 是否启用自动对齐
    uint16_t calibration_steps;         // 校准所需步数
} filter_config_t;

/**
 * @brief 处理结果状态
 */
typedef enum
{
    PROCESS_RESULT_NO_OUTPUT = 0,       // 无输出（累积中）
    PROCESS_RESULT_SIGNAL_READY,        // 信号准备就绪
    PROCESS_RESULT_ERROR_OVERFLOW,      // 累积溢出错误
    PROCESS_RESULT_ERROR_DEVIATION,     // 步数偏差过大
    PROCESS_RESULT_ERROR_INCONSISTENT   // 信号不一致
} process_result_t;

/**
 * @brief 预处理统计信息
 */
typedef struct
{
    uint32_t total_steps_processed;     // 处理的总步数
    uint32_t signals_generated;        // 生成的信号数
    uint32_t overflow_errors;          // 溢出错误数
    uint32_t deviation_errors;         // 偏差错误数
    uint32_t inconsistent_signals;     // 不一致信号数
    uint32_t adaptive_corrections;     // 自适应修正次数
    float average_steps_per_signal;    // 平均每信号步数
    uint32_t max_accumulated_steps;    // 最大累积步数
} preprocessor_stats_t;

/**
 * @brief 防抖滤波器状态
 */
typedef struct
{
    uint8_t *sample_buffer;             // 采样缓冲区
    uint8_t buffer_size;                // 缓冲区大小
    uint8_t current_index;              // 当前索引
    uint8_t valid_samples;              // 有效采样数
    uint8_t last_stable_value;          // 最后稳定值
    bool is_stable;                     // 是否稳定
    bool use_external_buffer;           // 是否使用外部缓冲区
} debounce_filter_t;

/**
 * @brief 输入预处理器结构
 * 在电机控制中断中处理每一步的信号：
 * - 电机步进到编码器单位的累积换算
 * - 防抖滤波（消除机械抖动）
 * - 累积误差管理
 */
typedef struct input_preprocessor_s
{
    // 配置参数
    conversion_config_t conversion_config;
    filter_config_t filter_config;
    
    // 换算状态
    uint32_t accumulated_steps;         // 累积的电机步数
    int32_t step_error_accumulator;     // 步数误差累积器
    uint16_t target_steps_per_unit;     // 目标步数每单位（可动态调整）
    
    // 防抖滤波器
    debounce_filter_t *debounce_filter;
    
    // 统计信息
    preprocessor_stats_t stats;
    
    // 内部状态
    bool is_initialized;                // 是否已初始化
    uint8_t last_processed_signal;      // 最后处理的信号
    uint32_t calibration_step_count;    // 校准步数计数
    bool calibration_mode;              // 是否在校准模式
} input_preprocessor_t;

/**
 * @brief 创建输入预处理器（动态内存版本）
 * @param conversion_config 换算配置参数
 * @param filter_config 滤波配置参数
 * @return 输入预处理器指针，失败返回NULL
 */
input_preprocessor_t *input_preprocessor_create(const conversion_config_t *conversion_config,
                                               const filter_config_t *filter_config);

/**
 * @brief 创建输入预处理器（静态缓冲区版本）
 * @param conversion_config 换算配置参数
 * @param filter_config 滤波配置参数
 * @param debounce_buffer 预分配的防抖缓冲区
 * @param buffer_size 缓冲区大小
 * @return 输入预处理器指针，失败返回NULL
 */
input_preprocessor_t *input_preprocessor_create_with_buffer(const conversion_config_t *conversion_config,
                                                           const filter_config_t *filter_config,
                                                           uint8_t *debounce_buffer, uint8_t buffer_size);

/**
 * @brief 销毁输入预处理器
 * @param preprocessor 输入预处理器指针
 */
void input_preprocessor_destroy(input_preprocessor_t *preprocessor);

/**
 * @brief 处理电机步进
 * @param preprocessor 输入预处理器指针
 * @param steps 步数
 * @param direction 方向 (1=前进, -1=后退)
 * @param raw_signal 原始编码器信号
 * @param processed_signal 输出参数：处理后的信号
 * @return 处理结果状态
 */
process_result_t input_preprocessor_process_steps(input_preprocessor_t *preprocessor, uint16_t steps, 
                                                  int8_t direction, uint8_t raw_signal, uint8_t *processed_signal);

/**
 * @brief 直接处理编码器信号（跳过步进换算）
 * @param preprocessor 输入预处理器指针
 * @param raw_signal 原始编码器信号
 * @param processed_signal 输出参数：处理后的信号
 * @return 处理结果状态
 */
process_result_t input_preprocessor_process_signal(input_preprocessor_t *preprocessor, uint8_t raw_signal, 
                                                   uint8_t *processed_signal);

/**
 * @brief 重置预处理器状态
 * @param preprocessor 输入预处理器指针
 */
void input_preprocessor_reset(input_preprocessor_t *preprocessor);

/**
 * @brief 进入校准模式
 * @param preprocessor 输入预处理器指针
 * @return 是否成功
 */
bool input_preprocessor_start_calibration(input_preprocessor_t *preprocessor);

/**
 * @brief 完成校准
 * @param preprocessor 输入预处理器指针
 * @param new_steps_per_unit 输出参数：校准得到的新步数每单位
 * @return 是否成功
 */
bool input_preprocessor_finish_calibration(input_preprocessor_t *preprocessor, uint16_t *new_steps_per_unit);

/**
 * @brief 获取统计信息
 * @param preprocessor 输入预处理器指针
 * @param stats 输出参数：统计信息
 */
void input_preprocessor_get_stats(const input_preprocessor_t *preprocessor, preprocessor_stats_t *stats);

/**
 * @brief 清零统计信息
 * @param preprocessor 输入预处理器指针
 */
void input_preprocessor_clear_stats(input_preprocessor_t *preprocessor);

/**
 * @brief 获取当前累积步数
 * @param preprocessor 输入预处理器指针
 * @return 当前累积步数
 */
uint32_t input_preprocessor_get_accumulated_steps(const input_preprocessor_t *preprocessor);

/**
 * @brief 设置自适应修正参数
 * @param preprocessor 输入预处理器指针
 * @param enable 是否启用
 * @param tolerance_ratio 容差比例
 * @return 是否成功
 */
bool input_preprocessor_set_adaptive_correction(input_preprocessor_t *preprocessor, bool enable, float tolerance_ratio);

#ifdef __cplusplus
}
#endif

#endif // INPUT_PREPROCESSOR_H
