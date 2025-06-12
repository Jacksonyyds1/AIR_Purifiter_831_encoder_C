#include "position_tracker.h"
#include <stdlib.h>
#include <string.h>

// 外部日志级别变量
extern int absolute_encoder_log_level;

// 常量定义
#define MAX_CONSECUTIVE_FAILURES 3

// 内部辅助函数声明
static uint16_t position_tracker_calculate_next_position(const position_tracker_t *tracker, uint16_t current_pos, int8_t direction);
static uint16_t position_tracker_calculate_previous_position(const position_tracker_t *tracker, uint16_t current_pos, int8_t direction);
static validation_result_t position_tracker_validate_position(position_tracker_t *tracker, uint8_t bit, int8_t direction);
static bool position_tracker_attempt_recovery(position_tracker_t *tracker, uint8_t bit);

// 实现

position_tracker_t *position_tracker_create(encoder_map_t *encoder_map, const uint8_t *pattern,
                                            uint16_t pattern_length, uint8_t max_validation_length)
{
    if (!encoder_map || !pattern || pattern_length == 0 || max_validation_length == 0)
    {
        LOGE(absolute_encoder_log_level, "Invalid parameters: encoder_map=%p, pattern=%p, pattern_length=%d, max_validation_length=%d",
             encoder_map, pattern, pattern_length, max_validation_length);
        return NULL;
    }

    position_tracker_t *tracker = (position_tracker_t *)malloc(sizeof(position_tracker_t));
    if (!tracker)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate position_tracker_t");
        return NULL;
    }

    // 分配验证缓冲区
    tracker->validation_buffer = (uint8_t *)malloc(max_validation_length);
    if (!tracker->validation_buffer)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate validation buffer");
        free(tracker);
        return NULL;
    }

    // 初始化结构体成员
    tracker->encoder_map = encoder_map;
    tracker->pattern = pattern;
    tracker->pattern_length = pattern_length;
    tracker->state = TRACKER_STATE_UNINITIALIZED;
    tracker->current_position = 0;
    tracker->movement_direction = 0;
    tracker->confidence_level = 0;
    tracker->validation_buffer_size = 0;
    tracker->max_validation_length = max_validation_length;
    tracker->validation_threshold = 3;
    tracker->recovery_attempts = 0;
    tracker->max_recovery_attempts = 3;
    tracker->use_external_buffer = false;

    // 清零统计信息
    position_tracker_clear_stats(tracker);

    LOGD(absolute_encoder_log_level, "PositionTracker initialized: pattern_length=%d, max_validation_length=%d",
         pattern_length, max_validation_length);

    return tracker;
}

position_tracker_t *position_tracker_create_with_buffer(encoder_map_t *encoder_map, const uint8_t *pattern,
                                                        uint16_t pattern_length, uint8_t *validation_buffer,
                                                        uint8_t max_validation_length)
{
    if (!encoder_map || !pattern || pattern_length == 0 || !validation_buffer || max_validation_length == 0)
    {
        LOGE(absolute_encoder_log_level, "Invalid parameters: encoder_map=%p, pattern=%p, pattern_length=%d, validation_buffer=%p, max_validation_length=%d",
             encoder_map, pattern, pattern_length, validation_buffer, max_validation_length);
        return NULL;
    }

    position_tracker_t *tracker = (position_tracker_t *)malloc(sizeof(position_tracker_t));
    if (!tracker)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate position_tracker_t");
        return NULL;
    }

    // 初始化结构体成员（使用外部缓冲区）
    tracker->encoder_map = encoder_map;
    tracker->pattern = pattern;
    tracker->pattern_length = pattern_length;
    tracker->state = TRACKER_STATE_UNINITIALIZED;
    tracker->current_position = 0;
    tracker->movement_direction = 0;
    tracker->confidence_level = 0;
    tracker->validation_buffer = validation_buffer;
    tracker->validation_buffer_size = 0;
    tracker->max_validation_length = max_validation_length;
    tracker->validation_threshold = 3;
    tracker->recovery_attempts = 0;
    tracker->max_recovery_attempts = 3;
    tracker->use_external_buffer = true;

    // 清零统计信息
    position_tracker_clear_stats(tracker);

    LOGD(absolute_encoder_log_level, "PositionTracker initialized with external buffer: pattern_length=%d, max_validation_length=%d",
         pattern_length, max_validation_length);

    return tracker;
}

void position_tracker_destroy(position_tracker_t *tracker)
{
    if (!tracker)
        return;

    // 只有内部分配的缓冲区才需要释放
    if (!tracker->use_external_buffer && tracker->validation_buffer)
    {
        free(tracker->validation_buffer);
    }

    free(tracker);
}

bool position_tracker_set_initial_position(position_tracker_t *tracker, uint16_t position, int8_t direction)
{
    if (!tracker || position >= tracker->pattern_length)
    {
        LOGE(absolute_encoder_log_level, "Invalid parameters: tracker=%p, position=%d, pattern_length=%d",
             tracker, position, tracker ? tracker->pattern_length : 0);
        return false;
    }

    tracker->current_position = position;
    tracker->movement_direction = direction;
    tracker->state = TRACKER_STATE_TRACKING;
    tracker->confidence_level = 100;
    tracker->validation_buffer_size = 0;
    tracker->recovery_attempts = 0;

    LOGD(absolute_encoder_log_level, "Position tracker initialized: position=%d, direction=%d",
         position, direction);

    return true;
}

validation_result_t position_tracker_update_position(position_tracker_t *tracker, uint8_t bit, int8_t direction,
                                                     uint16_t *new_position, uint8_t *confidence)
{
    if (!tracker || !new_position || !confidence)
        return VALIDATION_RESULT_INVALID;

    if (tracker->state == TRACKER_STATE_UNINITIALIZED)
    {
        LOGE(absolute_encoder_log_level, "Tracker not initialized");
        return VALIDATION_RESULT_INVALID;
    }

    tracker->total_updates++;
    tracker->movement_direction = direction;

    // 计算下一个预期位置
    uint16_t expected_position = position_tracker_calculate_next_position(tracker, tracker->current_position, direction);

    // 简单验证：检查预期位置的位值是否匹配
    if (tracker->pattern[expected_position] == bit)
    {
        // 匹配成功，更新位置
        tracker->current_position = expected_position;
        *new_position = tracker->current_position;
        
        // 增加置信度
        if (tracker->confidence_level < 100)
        {
            tracker->confidence_level += 10;
            if (tracker->confidence_level > 100)
                tracker->confidence_level = 100;
        }
        
        *confidence = tracker->confidence_level;
        tracker->valid_updates++;
        tracker->state = TRACKER_STATE_TRACKING;
        tracker->recovery_attempts = 0;

        LOGD(absolute_encoder_log_level, "Position updated to %u, bit=%d, confidence=%d", 
             tracker->current_position, bit, tracker->confidence_level);
        
        return VALIDATION_RESULT_VALID;
    }
    else
    {
        // 不匹配，降低置信度
        if (tracker->confidence_level > 20)
        {
            tracker->confidence_level -= 20;
        }
        else
        {
            tracker->confidence_level = 0;
        }

        *new_position = tracker->current_position; // 保持当前位置
        *confidence = tracker->confidence_level;

        LOGD(absolute_encoder_log_level, "Position validation failed: expected bit %d at position %u, got %d, confidence=%d",
             tracker->pattern[expected_position], expected_position, bit, tracker->confidence_level);

        // 如果置信度过低，尝试恢复
        if (tracker->confidence_level == 0)
        {
            tracker->state = TRACKER_STATE_LOST;
            LOGW(absolute_encoder_log_level, "Position tracking lost");

            // 尝试恢复
            if (position_tracker_attempt_recovery(tracker, bit))
            {
                tracker->recovery_success_count++;
                *new_position = tracker->current_position;
                *confidence = tracker->confidence_level;
                return VALIDATION_RESULT_VALID;
            }
            else
            {
                tracker->lost_tracking_count++;
                return VALIDATION_RESULT_INVALID;
            }
        }
        else
        {
            // 容错范围内，返回不确定
            return VALIDATION_RESULT_UNCERTAIN;
        }
    }
}

bool position_tracker_force_set_position(position_tracker_t *tracker, uint16_t position, bool reset_confidence)
{
    if (!tracker || position >= tracker->pattern_length)
        return false;

    tracker->current_position = position;
    tracker->state = TRACKER_STATE_TRACKING;
    tracker->recovery_attempts = 0;
    
    if (reset_confidence)
    {
        tracker->confidence_level = 100;
    }

    LOGI(absolute_encoder_log_level, "Position force set to %d, confidence=%d", position, tracker->confidence_level);
    return true;
}

bool position_tracker_get_current_info(const position_tracker_t *tracker, uint16_t *position, 
                                       uint8_t *confidence, tracker_state_t *state)
{
    if (!tracker || !position || !confidence || !state)
        return false;

    *position = tracker->current_position;
    *confidence = tracker->confidence_level;
    *state = tracker->state;
    return true;
}

void position_tracker_reset(position_tracker_t *tracker)
{
    if (!tracker)
        return;

    tracker->state = TRACKER_STATE_UNINITIALIZED;
    tracker->current_position = 0;
    tracker->movement_direction = 0;
    tracker->confidence_level = 0;
    tracker->validation_buffer_size = 0;
    tracker->recovery_attempts = 0;

    LOGD(absolute_encoder_log_level, "Position tracker reset");
}

void position_tracker_get_stats(const position_tracker_t *tracker, uint32_t *total_updates, 
                                uint32_t *valid_updates, uint32_t *lost_count, uint32_t *recovery_count)
{
    if (!tracker)
        return;

    if (total_updates)
        *total_updates = tracker->total_updates;
    if (valid_updates)
        *valid_updates = tracker->valid_updates;
    if (lost_count)
        *lost_count = tracker->lost_tracking_count;
    if (recovery_count)
        *recovery_count = tracker->recovery_success_count;
}

void position_tracker_clear_stats(position_tracker_t *tracker)
{
    if (!tracker)
        return;

    tracker->total_updates = 0;
    tracker->valid_updates = 0;
    tracker->lost_tracking_count = 0;
    tracker->recovery_success_count = 0;
}

void position_tracker_set_validation_params(position_tracker_t *tracker, uint8_t validation_threshold, 
                                            uint8_t max_recovery_attempts)
{
    if (!tracker)
        return;

    tracker->validation_threshold = validation_threshold;
    tracker->max_recovery_attempts = max_recovery_attempts;

    LOGD(absolute_encoder_log_level, "Validation params set: threshold=%d, max_recovery=%d",
         validation_threshold, max_recovery_attempts);
}

// 内部辅助函数实现

static uint16_t position_tracker_calculate_next_position(const position_tracker_t *tracker, uint16_t current_pos, int8_t direction)
{
    if (!tracker)
        return 0;

    if (direction > 0)
    {
        // 正向
        return (current_pos + 1) % tracker->pattern_length;
    }
    else if (direction < 0)
    {
        // 反向，注意处理下溢
        return (current_pos == 0) ? (tracker->pattern_length - 1) : (current_pos - 1);
    }
    else
    {
        // 静止
        return current_pos;
    }
}

static uint16_t position_tracker_calculate_previous_position(const position_tracker_t *tracker, uint16_t current_pos, int8_t direction)
{
    if (!tracker)
        return 0;

    if (direction > 0)
    {
        // 正向移动时，前一个位置是向后一位
        return (current_pos == 0) ? (tracker->pattern_length - 1) : (current_pos - 1);
    }
    else if (direction < 0)
    {
        // 反向移动时，前一个位置是向前一位
        return (current_pos + 1) % tracker->pattern_length;
    }
    else
    {
        // 静止
        return current_pos;
    }
}

static validation_result_t position_tracker_validate_position(position_tracker_t *tracker, uint8_t bit, int8_t direction)
{
    if (!tracker)
        return VALIDATION_RESULT_INVALID;

    // 添加位到验证缓冲区
    if (tracker->validation_buffer_size < tracker->max_validation_length)
    {
        tracker->validation_buffer[tracker->validation_buffer_size] = bit;
        tracker->validation_buffer_size++;
    }
    else
    {
        // 缓冲区满，移除最早的位
        for (uint8_t i = 0; i < tracker->max_validation_length - 1; i++)
        {
            tracker->validation_buffer[i] = tracker->validation_buffer[i + 1];
        }
        tracker->validation_buffer[tracker->max_validation_length - 1] = bit;
    }

    // 简单验证：检查缓冲区内容是否与当前位置附近的图案匹配
    uint8_t match_count = 0;
    uint16_t check_position = tracker->current_position;

    for (uint8_t i = 0; i < tracker->validation_buffer_size; i++)
    {
        if (tracker->pattern[check_position] == tracker->validation_buffer[i])
        {
            match_count++;
        }
        check_position = position_tracker_calculate_next_position(tracker, check_position, direction);
    }

    // 根据匹配率判断验证结果
    if (match_count >= tracker->validation_threshold)
    {
        return VALIDATION_RESULT_VALID;
    }
    else if (match_count > 0)
    {
        return VALIDATION_RESULT_UNCERTAIN;
    }
    else
    {
        return VALIDATION_RESULT_INVALID;
    }
}

static bool position_tracker_attempt_recovery(position_tracker_t *tracker, uint8_t bit)
{
    if (!tracker || tracker->recovery_attempts >= tracker->max_recovery_attempts)
        return false;

    tracker->recovery_attempts++;
    tracker->state = TRACKER_STATE_RECOVERING;

    LOGD(absolute_encoder_log_level, "Attempting recovery, attempt %d/%d", 
         tracker->recovery_attempts, tracker->max_recovery_attempts);

    // 尝试简单恢复：检查当前位置是否匹配
    if (tracker->pattern[tracker->current_position] == bit)
    {
        // 位置没变，可能是运动停止了
        tracker->confidence_level = 50;
        tracker->state = TRACKER_STATE_TRACKING;
        LOGI(absolute_encoder_log_level, "Recovery successful: position unchanged");
        return true;
    }

    // 尝试在附近位置搜索
    for (uint16_t offset = 1; offset <= 3 && offset < tracker->pattern_length; offset++)
    {
        // 向前搜索
        uint16_t forward_pos = (tracker->current_position + offset) % tracker->pattern_length;
        if (tracker->pattern[forward_pos] == bit)
        {
            tracker->current_position = forward_pos;
            tracker->confidence_level = 30;
            tracker->state = TRACKER_STATE_TRACKING;
            LOGI(absolute_encoder_log_level, "Recovery successful: found at forward offset %d, new position %d", 
                 offset, forward_pos);
            return true;
        }

        // 向后搜索
        uint16_t backward_pos = (tracker->current_position - offset + tracker->pattern_length) % tracker->pattern_length;
        if (tracker->pattern[backward_pos] == bit)
        {
            tracker->current_position = backward_pos;
            tracker->confidence_level = 30;
            tracker->state = TRACKER_STATE_TRACKING;
            LOGI(absolute_encoder_log_level, "Recovery successful: found at backward offset %d, new position %d", 
                 offset, backward_pos);
            return true;
        }
    }

    LOGW(absolute_encoder_log_level, "Recovery attempt %d failed", tracker->recovery_attempts);
    return false;
}
