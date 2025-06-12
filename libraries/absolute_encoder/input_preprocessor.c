#include "input_preprocessor.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// 外部日志级别变量
extern int absolute_encoder_log_level;

// 内部辅助函数声明
static debounce_filter_t *create_debounce_filter(uint8_t buffer_size, uint8_t *external_buffer);
static void destroy_debounce_filter(debounce_filter_t *filter);
static bool debounce_filter_process(debounce_filter_t *filter, uint8_t signal, uint8_t *output);
static uint16_t apply_tolerance_adjustment(input_preprocessor_t *preprocessor);
static void update_average_steps(input_preprocessor_t *preprocessor, uint16_t steps_taken);
static bool check_consecutive_signals(input_preprocessor_t *preprocessor, int8_t direction);
static void update_preprocessor_stats(input_preprocessor_t *preprocessor, process_result_t result);

// 实现

input_preprocessor_t *input_preprocessor_create(const conversion_config_t *conversion_config,
                                               const filter_config_t *filter_config)
{
    if (!conversion_config || !filter_config)
    {
        LOGE(absolute_encoder_log_level, "Invalid parameters: conversion_config=%p, filter_config=%p",
             conversion_config, filter_config);
        return NULL;
    }

    input_preprocessor_t *preprocessor = (input_preprocessor_t *)malloc(sizeof(input_preprocessor_t));
    if (!preprocessor)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate input_preprocessor_t");
        return NULL;
    }

    // 复制配置
    preprocessor->conversion_config = *conversion_config;
    preprocessor->filter_config = *filter_config;

    // 创建防抖滤波器
    preprocessor->debounce_filter = create_debounce_filter(filter_config->debounce_samples, NULL);
    if (!preprocessor->debounce_filter)
    {
        LOGE(absolute_encoder_log_level, "Failed to create debounce filter");
        free(preprocessor);
        return NULL;
    }

    // 初始化状态
    preprocessor->accumulated_steps = 0;
    preprocessor->step_error_accumulator = 0;
    preprocessor->target_steps_per_unit = conversion_config->motor_steps_per_unit;
    preprocessor->is_initialized = true;
    preprocessor->last_processed_signal = 0;
    preprocessor->calibration_step_count = 0;
    preprocessor->calibration_mode = false;

    // 清零统计信息
    input_preprocessor_clear_stats(preprocessor);

    LOGD(absolute_encoder_log_level, 
         "InputPreprocessor initialized with motor_steps_per_unit=%d, debounce_samples=%d",
         conversion_config->motor_steps_per_unit, filter_config->debounce_samples);

    return preprocessor;
}

input_preprocessor_t *input_preprocessor_create_with_buffer(const conversion_config_t *conversion_config,
                                                           const filter_config_t *filter_config,
                                                           uint8_t *debounce_buffer, uint8_t buffer_size)
{
    if (!conversion_config || !filter_config || !debounce_buffer || buffer_size == 0)
    {
        LOGE(absolute_encoder_log_level, 
             "Invalid parameters: conversion_config=%p, filter_config=%p, debounce_buffer=%p, buffer_size=%d",
             conversion_config, filter_config, debounce_buffer, buffer_size);
        return NULL;
    }

    input_preprocessor_t *preprocessor = (input_preprocessor_t *)malloc(sizeof(input_preprocessor_t));
    if (!preprocessor)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate input_preprocessor_t");
        return NULL;
    }

    // 复制配置
    preprocessor->conversion_config = *conversion_config;
    preprocessor->filter_config = *filter_config;

    // 创建防抖滤波器（使用外部缓冲区）
    preprocessor->debounce_filter = create_debounce_filter(buffer_size, debounce_buffer);
    if (!preprocessor->debounce_filter)
    {
        LOGE(absolute_encoder_log_level, "Failed to create debounce filter with external buffer");
        free(preprocessor);
        return NULL;
    }

    // 初始化状态
    preprocessor->accumulated_steps = 0;
    preprocessor->step_error_accumulator = 0;
    preprocessor->target_steps_per_unit = conversion_config->motor_steps_per_unit;
    preprocessor->is_initialized = true;
    preprocessor->last_processed_signal = 0;
    preprocessor->calibration_step_count = 0;
    preprocessor->calibration_mode = false;

    // 清零统计信息
    input_preprocessor_clear_stats(preprocessor);

    LOGD(absolute_encoder_log_level, 
         "InputPreprocessor initialized with external buffer, motor_steps_per_unit=%d, debounce_samples=%d",
         conversion_config->motor_steps_per_unit, buffer_size);

    return preprocessor;
}

void input_preprocessor_destroy(input_preprocessor_t *preprocessor)
{
    if (!preprocessor)
        return;

    // 销毁防抖滤波器
    destroy_debounce_filter(preprocessor->debounce_filter);

    free(preprocessor);
}

process_result_t input_preprocessor_process_steps(input_preprocessor_t *preprocessor, uint16_t steps, 
                                                  int8_t direction, uint8_t raw_signal, uint8_t *processed_signal)
{
    if (!preprocessor || !processed_signal)
        return PROCESS_RESULT_ERROR_INCONSISTENT;

    if (!preprocessor->is_initialized)
        return PROCESS_RESULT_ERROR_INCONSISTENT;

    preprocessor->stats.total_steps_processed += steps;

    // 校准模式处理
    if (preprocessor->calibration_mode)
    {
        preprocessor->calibration_step_count += steps;
        if (preprocessor->calibration_step_count >= preprocessor->filter_config.calibration_steps)
        {
            preprocessor->calibration_mode = false;
            LOGI(absolute_encoder_log_level, "Calibration completed after %d steps", 
                 preprocessor->calibration_step_count);
        }
    }

    // 累积步数
    if (direction > 0)
    {
        preprocessor->accumulated_steps += steps;
    }
    else if (direction < 0)
    {
        if (preprocessor->accumulated_steps >= steps)
        {
            preprocessor->accumulated_steps -= steps;
        }
        else
        {
            // 处理反向超出的情况
            preprocessor->accumulated_steps = 0;
        }
    }

    // 检查是否达到单位长度
    uint16_t unit_threshold = apply_tolerance_adjustment(preprocessor);
    
    if (preprocessor->accumulated_steps >= unit_threshold)
    {
        // 应用防抖滤波
        if (debounce_filter_process(preprocessor->debounce_filter, raw_signal, processed_signal))
        {
            uint16_t actual_steps = preprocessor->accumulated_steps;
            preprocessor->accumulated_steps = 0;
            
            // 更新统计信息
            update_average_steps(preprocessor, actual_steps);
            preprocessor->stats.signals_generated++;
            
            // 检查步数偏差
            if (abs((int)actual_steps - (int)preprocessor->target_steps_per_unit) > 
                preprocessor->conversion_config.max_step_deviation)
            {
                preprocessor->stats.deviation_errors++;
                return PROCESS_RESULT_ERROR_DEVIATION;
            }

            preprocessor->last_processed_signal = *processed_signal;
            update_preprocessor_stats(preprocessor, PROCESS_RESULT_SIGNAL_READY);
            return PROCESS_RESULT_SIGNAL_READY;
        }
        else
        {
            // 信号被防抖过滤
            return PROCESS_RESULT_NO_OUTPUT;
        }
    }

    // 检查累积溢出
    if (preprocessor->accumulated_steps > unit_threshold * 2)
    {
        preprocessor->stats.overflow_errors++;
        preprocessor->accumulated_steps = 0;  // 重置以防止进一步溢出
        return PROCESS_RESULT_ERROR_OVERFLOW;
    }

    return PROCESS_RESULT_NO_OUTPUT;
}

process_result_t input_preprocessor_process_signal(input_preprocessor_t *preprocessor, uint8_t raw_signal, 
                                                   uint8_t *processed_signal)
{
    if (!preprocessor || !processed_signal)
        return PROCESS_RESULT_ERROR_INCONSISTENT;

    if (!preprocessor->is_initialized)
        return PROCESS_RESULT_ERROR_INCONSISTENT;

    // 直接应用防抖滤波
    if (debounce_filter_process(preprocessor->debounce_filter, raw_signal, processed_signal))
    {
        preprocessor->stats.signals_generated++;
        preprocessor->last_processed_signal = *processed_signal;
        update_preprocessor_stats(preprocessor, PROCESS_RESULT_SIGNAL_READY);
        return PROCESS_RESULT_SIGNAL_READY;
    }

    return PROCESS_RESULT_NO_OUTPUT;
}

void input_preprocessor_reset(input_preprocessor_t *preprocessor)
{
    if (!preprocessor)
        return;

    preprocessor->accumulated_steps = 0;
    preprocessor->step_error_accumulator = 0;
    preprocessor->target_steps_per_unit = preprocessor->conversion_config.motor_steps_per_unit;
    preprocessor->last_processed_signal = 0;
    preprocessor->calibration_step_count = 0;
    preprocessor->calibration_mode = false;

    // 重置防抖滤波器
    if (preprocessor->debounce_filter)
    {
        preprocessor->debounce_filter->current_index = 0;
        preprocessor->debounce_filter->valid_samples = 0;
        preprocessor->debounce_filter->is_stable = false;
        memset(preprocessor->debounce_filter->sample_buffer, 0, preprocessor->debounce_filter->buffer_size);
    }

    input_preprocessor_clear_stats(preprocessor);

    LOGD(absolute_encoder_log_level, "InputPreprocessor reset");
}

bool input_preprocessor_start_calibration(input_preprocessor_t *preprocessor)
{
    if (!preprocessor)
        return false;

    preprocessor->calibration_mode = true;
    preprocessor->calibration_step_count = 0;

    LOGI(absolute_encoder_log_level, "Calibration started, target steps: %d", 
         preprocessor->filter_config.calibration_steps);
    return true;
}

bool input_preprocessor_finish_calibration(input_preprocessor_t *preprocessor, uint16_t *new_steps_per_unit)
{
    if (!preprocessor || !new_steps_per_unit)
        return false;

    if (preprocessor->calibration_mode)
    {
        preprocessor->calibration_mode = false;
    }

    // 计算新的步数每单位值（基于平均值）
    if (preprocessor->stats.average_steps_per_signal > 0)
    {
        *new_steps_per_unit = (uint16_t)preprocessor->stats.average_steps_per_signal;
        preprocessor->target_steps_per_unit = *new_steps_per_unit;
        
        LOGI(absolute_encoder_log_level, "Calibration finished, new steps per unit: %d", *new_steps_per_unit);
        return true;
    }

    return false;
}

void input_preprocessor_get_stats(const input_preprocessor_t *preprocessor, preprocessor_stats_t *stats)
{
    if (!preprocessor || !stats)
        return;

    *stats = preprocessor->stats;
}

void input_preprocessor_clear_stats(input_preprocessor_t *preprocessor)
{
    if (!preprocessor)
        return;

    memset(&preprocessor->stats, 0, sizeof(preprocessor_stats_t));
}

uint32_t input_preprocessor_get_accumulated_steps(const input_preprocessor_t *preprocessor)
{
    if (!preprocessor)
        return 0;

    return preprocessor->accumulated_steps;
}

bool input_preprocessor_set_adaptive_correction(input_preprocessor_t *preprocessor, bool enable, float tolerance_ratio)
{
    if (!preprocessor)
        return false;

    preprocessor->conversion_config.enable_adaptive_correction = enable;
    preprocessor->conversion_config.step_tolerance_ratio = tolerance_ratio;

    LOGD(absolute_encoder_log_level, "Adaptive correction set: enable=%d, tolerance_ratio=%.3f", 
         enable, tolerance_ratio);
    return true;
}

// 内部辅助函数实现

static debounce_filter_t *create_debounce_filter(uint8_t buffer_size, uint8_t *external_buffer)
{
    if (buffer_size == 0)
        return NULL;

    debounce_filter_t *filter = (debounce_filter_t *)malloc(sizeof(debounce_filter_t));
    if (!filter)
        return NULL;

    if (external_buffer)
    {
        filter->sample_buffer = external_buffer;
        filter->use_external_buffer = true;
    }
    else
    {
        filter->sample_buffer = (uint8_t *)malloc(buffer_size);
        if (!filter->sample_buffer)
        {
            free(filter);
            return NULL;
        }
        filter->use_external_buffer = false;
    }

    filter->buffer_size = buffer_size;
    filter->current_index = 0;
    filter->valid_samples = 0;
    filter->last_stable_value = 0;
    filter->is_stable = false;

    memset(filter->sample_buffer, 0, buffer_size);
    return filter;
}

static void destroy_debounce_filter(debounce_filter_t *filter)
{
    if (!filter)
        return;

    if (!filter->use_external_buffer && filter->sample_buffer)
    {
        free(filter->sample_buffer);
    }

    free(filter);
}

static bool debounce_filter_process(debounce_filter_t *filter, uint8_t signal, uint8_t *output)
{
    if (!filter || !output)
        return false;

    // 添加新样本
    filter->sample_buffer[filter->current_index] = signal;
    filter->current_index = (filter->current_index + 1) % filter->buffer_size;
    
    if (filter->valid_samples < filter->buffer_size)
    {
        filter->valid_samples++;
    }

    // 需要足够的样本才能做决定
    if (filter->valid_samples < filter->buffer_size)
    {
        return false;
    }

    // 计算相同值的数量
    uint8_t count_0 = 0;
    uint8_t count_1 = 0;
    
    for (uint8_t i = 0; i < filter->buffer_size; i++)
    {
        if (filter->sample_buffer[i] == 0)
            count_0++;
        else if (filter->sample_buffer[i] == 1)
            count_1++;
    }

    // 多数决策
    uint8_t stable_value;
    if (count_1 > count_0)
    {
        stable_value = 1;
    }
    else
    {
        stable_value = 0;
    }

    // 检查是否与上次稳定值不同
    if (!filter->is_stable || stable_value != filter->last_stable_value)
    {
        filter->last_stable_value = stable_value;
        filter->is_stable = true;
        *output = stable_value;
        return true;
    }

    return false; // 没有变化，不输出
}

static uint16_t apply_tolerance_adjustment(input_preprocessor_t *preprocessor)
{
    if (!preprocessor || !preprocessor->conversion_config.enable_adaptive_correction)
    {
        return preprocessor->target_steps_per_unit;
    }

    float avg_steps = preprocessor->stats.average_steps_per_signal;
    if (avg_steps > 0.0f && preprocessor->stats.signals_generated >= 2)
    {
        float tolerance = preprocessor->target_steps_per_unit * preprocessor->conversion_config.step_tolerance_ratio;
        float adjusted_threshold = avg_steps;

        // 限制调整范围
        if (adjusted_threshold < preprocessor->target_steps_per_unit - tolerance)
        {
            adjusted_threshold = preprocessor->target_steps_per_unit - tolerance;
            preprocessor->stats.adaptive_corrections++;
        }
        else if (adjusted_threshold > preprocessor->target_steps_per_unit + tolerance)
        {
            adjusted_threshold = preprocessor->target_steps_per_unit + tolerance;
            preprocessor->stats.adaptive_corrections++;
        }

        return (uint16_t)adjusted_threshold;
    }

    return preprocessor->target_steps_per_unit;
}

static void update_average_steps(input_preprocessor_t *preprocessor, uint16_t steps_taken)
{
    if (!preprocessor)
        return;

    // 简单的指数移动平均
    float alpha = 0.1f; // 平滑因子
    if (preprocessor->stats.average_steps_per_signal == 0.0f)
    {
        preprocessor->stats.average_steps_per_signal = (float)steps_taken;
    }
    else
    {
        preprocessor->stats.average_steps_per_signal = 
            alpha * steps_taken + (1.0f - alpha) * preprocessor->stats.average_steps_per_signal;
    }

    if (steps_taken > preprocessor->stats.max_accumulated_steps)
    {
        preprocessor->stats.max_accumulated_steps = steps_taken;
    }
}

static bool check_consecutive_signals(input_preprocessor_t *preprocessor, int8_t direction)
{
    // 简化的连续信号检查
    (void)preprocessor;
    (void)direction;
    
    // 这里可以实现更复杂的连续信号检查逻辑
    return false;
}

static void update_preprocessor_stats(input_preprocessor_t *preprocessor, process_result_t result)
{
    if (!preprocessor)
        return;

    switch (result)
    {
        case PROCESS_RESULT_SIGNAL_READY:
            // 统计信息已在主处理函数中更新
            break;
        case PROCESS_RESULT_ERROR_OVERFLOW:
            // 溢出错误统计已在主处理函数中更新
            break;
        case PROCESS_RESULT_ERROR_DEVIATION:
            // 偏差错误统计已在主处理函数中更新
            break;
        case PROCESS_RESULT_ERROR_INCONSISTENT:
            preprocessor->stats.inconsistent_signals++;
            break;
        default:
            break;
    }
}
