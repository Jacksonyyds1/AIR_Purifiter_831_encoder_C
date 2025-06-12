#include "absolute_encoder.h"
#include <stdlib.h>
#include <string.h>

// 全局日志级别变量
int absolute_encoder_log_level = 0;

/**
 * 编码器图案句柄实现
 */
struct encoder_map_handle_s
{
    encoder_map_t *encoder_map;
};

/**
 * 编码器句柄实现
 */
struct encoder_handle_s
{
    // 共享的编码器图案
    encoder_map_handle_t map_handle;

    // 各功能模块
    input_preprocessor_t *input_processor;
    smart_search_manager_t *search_manager;
    position_tracker_t *position_tracker;

    // 配置和状态
    encoder_config_t config;
    encoder_state_t current_state;
    encoder_position_t current_position;
    encoder_stats_t stats;

    // 运行时状态
    uint32_t start_time;
    uint32_t last_signal_timestamp;
    bool debug_enabled;
};

// 内部辅助函数声明
static encoder_result_t handle_search_mode(encoder_handle_t handle, uint8_t signal_bit);
static encoder_result_t handle_tracking_mode(encoder_handle_t handle, uint8_t signal_bit);
static void update_runtime_stats(encoder_handle_t handle);
static void transition_to_tracking_mode(encoder_handle_t handle, uint32_t found_position);
static void transition_to_search_mode(encoder_handle_t handle);

// === 日志等级设置 ===

void absolute_encoder_set_log_level(int level)
{
    absolute_encoder_log_level = level;
}

// === 编码器图案对象管理 ===

encoder_map_handle_t create_encoder_map(const uint8_t *pattern_data, uint32_t pattern_length, uint8_t max_search_depth)
{
    if (!pattern_data || pattern_length == 0 || max_search_depth == 0)
    {
        return NULL;
    }

    encoder_map_handle_t map_handle = (encoder_map_handle_t)malloc(sizeof(struct encoder_map_handle_s));
    if (!map_handle)
    {
        return NULL;
    }

    map_handle->encoder_map = encoder_map_create(pattern_data, (uint16_t)pattern_length, max_search_depth);
    if (!map_handle->encoder_map)
    {
        free(map_handle);
        return NULL;
    }

    return map_handle;
}

encoder_map_handle_t create_encoder_map_with_pool(const uint8_t *pattern_data, uint32_t pattern_length, uint8_t max_search_depth, void *node_pool, uint16_t pool_size)
{
    if (!pattern_data || pattern_length == 0 || max_search_depth == 0 || !node_pool || pool_size == 0)
    {
        return NULL;
    }

    encoder_map_handle_t map_handle = (encoder_map_handle_t)malloc(sizeof(struct encoder_map_handle_s));
    if (!map_handle)
    {
        return NULL;
    }

    map_handle->encoder_map = encoder_map_create_with_pool(pattern_data, (uint16_t)pattern_length, max_search_depth, 
                                                          (tree_node_t *)node_pool, pool_size);
    if (!map_handle->encoder_map)
    {
        free(map_handle);
        return NULL;
    }

    return map_handle;
}

void destroy_encoder_map(encoder_map_handle_t map_handle)
{
    if (!map_handle)
        return;

    if (map_handle->encoder_map)
    {
        encoder_map_destroy(map_handle->encoder_map);
    }

    free(map_handle);
}

// === 编码器对象管理 ===

encoder_handle_t create_encoder(encoder_map_handle_t map_handle, const encoder_config_t *config)
{
    if (!map_handle || !config)
    {
        LOGE(absolute_encoder_log_level, "Invalid parameters: map_handle=%p, config=%p", map_handle, config);
        return NULL;
    }

    encoder_handle_t handle = (encoder_handle_t)malloc(sizeof(struct encoder_handle_s));
    if (!handle)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate encoder handle");
        return NULL;
    }

    // 初始化基本信息
    handle->map_handle = map_handle;
    handle->config = *config;
    handle->current_state = ENCODER_STATE_UNINITIALIZED;
    handle->debug_enabled = false;
    handle->start_time = 0;
    handle->last_signal_timestamp = 0;

    // 初始化位置信息
    memset(&handle->current_position, 0, sizeof(encoder_position_t));
    handle->current_position.state = ENCODER_STATE_UNINITIALIZED;

    // 初始化统计信息
    memset(&handle->stats, 0, sizeof(encoder_stats_t));

    // 创建输入预处理器
    conversion_config_t conv_config = {
        .motor_steps_per_unit = config->motor_steps_per_unit,
        .step_tolerance_ratio = 0.1f,
        .max_step_deviation = config->motor_steps_per_unit / 10,
        .enable_adaptive_correction = true,
        .enable_auto_alignment = false,
        .alignment_sample_count = 10,
        .alignment_tolerance = 0.05f
    };

    filter_config_t filter_config = {
        .debounce_samples = 3,
        .enable_direction_filter = false,
        .direction_filter_window = 5,
        .consistency_threshold = 10,
        .enable_pattern_validation = false,
        .enable_auto_alignment = false,
        .calibration_steps = 100
    };

    handle->input_processor = input_preprocessor_create(&conv_config, &filter_config);
    if (!handle->input_processor)
    {
        LOGE(absolute_encoder_log_level, "Failed to create input preprocessor");
        free(handle);
        return NULL;
    }

    // 创建智能搜索管理器
    handle->search_manager = smart_search_manager_create(map_handle->encoder_map, 32);
    if (!handle->search_manager)
    {
        LOGE(absolute_encoder_log_level, "Failed to create search manager");
        input_preprocessor_destroy(handle->input_processor);
        free(handle);
        return NULL;
    }

    // 创建位置跟踪器
    uint16_t pattern_length = encoder_map_get_pattern_length(map_handle->encoder_map);
    const uint8_t *pattern = map_handle->encoder_map->pattern; // 直接访问模式数据
    handle->position_tracker = position_tracker_create(map_handle->encoder_map, pattern, pattern_length, 10);
    if (!handle->position_tracker)
    {
        LOGE(absolute_encoder_log_level, "Failed to create position tracker");
        smart_search_manager_destroy(handle->search_manager);
        input_preprocessor_destroy(handle->input_processor);
        free(handle);
        return NULL;
    }

    // 初始化时间戳
    if (config->sys_tick_get)
    {
        handle->start_time = config->sys_tick_get();
    }

    // 设置初始状态为搜索模式
    transition_to_search_mode(handle);

    LOGI(absolute_encoder_log_level, "Encoder created successfully with motor_steps_per_unit=%d", 
         config->motor_steps_per_unit);

    return handle;
}

void destroy_encoder(encoder_handle_t handle)
{
    if (!handle)
        return;

    // 销毁各个模块
    if (handle->position_tracker)
    {
        position_tracker_destroy(handle->position_tracker);
    }

    if (handle->search_manager)
    {
        smart_search_manager_destroy(handle->search_manager);
    }

    if (handle->input_processor)
    {
        input_preprocessor_destroy(handle->input_processor);
    }

    free(handle);
}

// === 主要处理接口 ===

encoder_result_t process_encoder_signal(encoder_handle_t handle, uint8_t signal_bit, encoder_position_t *position_info)
{
    if (!handle || !position_info)
    {
        return ENCODER_RESULT_ERROR_INVALID_SIGNAL;
    }

    handle->stats.total_signals_processed++;
    
    // 更新时间戳
    if (handle->config.sys_tick_get)
    {
        handle->last_signal_timestamp = handle->config.sys_tick_get();
    }

    encoder_result_t result = ENCODER_RESULT_OK;

    // 根据当前状态处理信号
    switch (handle->current_state)
    {
        case ENCODER_STATE_SEARCHING:
            result = handle_search_mode(handle, signal_bit);
            break;

        case ENCODER_STATE_TRACKING:
            result = handle_tracking_mode(handle, signal_bit);
            break;

        case ENCODER_STATE_ERROR:
            // 错误状态，尝试重新进入搜索模式
            transition_to_search_mode(handle);
            result = ENCODER_RESULT_ERROR_INVALID_SIGNAL;
            break;

        case ENCODER_STATE_UNINITIALIZED:
        default:
            transition_to_search_mode(handle);
            result = ENCODER_RESULT_SEARCHING;
            break;
    }

    // 更新统计信息
    update_runtime_stats(handle);

    // 复制当前位置信息
    *position_info = handle->current_position;

    return result;
}

encoder_result_t process_motor_steps(encoder_handle_t handle, uint16_t steps, int8_t direction)
{
    if (!handle)
    {
        return ENCODER_RESULT_ERROR_INVALID_SIGNAL;
    }

    // 这里可以预处理电机步数，为后续的信号处理做准备
    // 目前只是简单地记录信息
    LOGV(absolute_encoder_log_level, "Motor steps: %d, direction: %d", steps, direction);

    return ENCODER_RESULT_OK;
}

bool get_current_position(encoder_handle_t handle, encoder_position_t *position_info)
{
    if (!handle || !position_info)
        return false;

    *position_info = handle->current_position;
    return true;
}

bool get_encoder_stats(encoder_handle_t handle, encoder_stats_t *stats)
{
    if (!handle || !stats)
        return false;

    // 更新运行时统计
    update_runtime_stats(handle);
    *stats = handle->stats;
    return true;
}

bool reset_encoder(encoder_handle_t handle)
{
    if (!handle)
        return false;

    // 重置各个模块
    if (handle->input_processor)
    {
        input_preprocessor_reset(handle->input_processor);
    }

    if (handle->search_manager)
    {
        smart_search_manager_reset(handle->search_manager);
    }

    if (handle->position_tracker)
    {
        position_tracker_reset(handle->position_tracker);
    }

    // 重置状态和统计
    memset(&handle->current_position, 0, sizeof(encoder_position_t));
    memset(&handle->stats, 0, sizeof(encoder_stats_t));
    
    // 重置时间戳
    if (handle->config.sys_tick_get)
    {
        handle->start_time = handle->config.sys_tick_get();
    }

    // 转换到搜索模式
    transition_to_search_mode(handle);

    LOGI(absolute_encoder_log_level, "Encoder reset completed");
    return true;
}

bool force_search_mode(encoder_handle_t handle)
{
    if (!handle)
        return false;

    transition_to_search_mode(handle);
    
    LOGI(absolute_encoder_log_level, "Forced to search mode");
    return true;
}

// === 内部辅助函数实现 ===

static encoder_result_t handle_search_mode(encoder_handle_t handle, uint8_t signal_bit)
{
    if (!handle)
        return ENCODER_RESULT_ERROR_INVALID_SIGNAL;

    handle->stats.search_attempts++;

    // 使用智能搜索管理器进行搜索
    uint16_t found_position;
    search_result_t search_result = smart_search_manager_add_bit_and_search(
        handle->search_manager, signal_bit, SEARCH_DIRECTION_FORWARD, &found_position);

    switch (search_result)
    {
        case SEARCH_RESULT_FOUND:
            // 找到位置，切换到跟踪模式
            transition_to_tracking_mode(handle, found_position);
            return ENCODER_RESULT_POSITION_FOUND;

        case SEARCH_RESULT_PARTIAL:
            // 部分匹配，继续搜索
            handle->current_position.state = ENCODER_STATE_SEARCHING;
            return ENCODER_RESULT_SEARCHING;

        case SEARCH_RESULT_NOT_FOUND:
        default:
            // 搜索失败，继续在搜索模式
            handle->current_position.state = ENCODER_STATE_SEARCHING;
            handle->stats.error_count++;
            return ENCODER_RESULT_SEARCHING;
    }
}

static encoder_result_t handle_tracking_mode(encoder_handle_t handle, uint8_t signal_bit)
{
    if (!handle)
        return ENCODER_RESULT_ERROR_INVALID_SIGNAL;

    handle->stats.tracking_updates++;

    // 使用位置跟踪器更新位置
    uint16_t new_position;
    uint8_t confidence;
    validation_result_t validation_result = position_tracker_update_position(
        handle->position_tracker, signal_bit, 1, &new_position, &confidence);

    switch (validation_result)
    {
        case VALIDATION_RESULT_VALID:
            // 位置有效，更新当前位置
            handle->current_position.absolute_position = new_position;
            handle->current_position.confidence_level = confidence;
            handle->current_position.state = ENCODER_STATE_TRACKING;
            return ENCODER_RESULT_TRACKING_UPDATED;

        case VALIDATION_RESULT_UNCERTAIN:
            // 不确定，保持跟踪模式但降低置信度
            handle->current_position.confidence_level = confidence;
            handle->current_position.state = ENCODER_STATE_TRACKING;
            return ENCODER_RESULT_TRACKING_UPDATED;

        case VALIDATION_RESULT_INVALID:
        default:
            // 跟踪失败，切换到搜索模式
            transition_to_search_mode(handle);
            handle->stats.error_count++;
            return ENCODER_RESULT_ERROR_LOST_TRACKING;
    }
}

static void update_runtime_stats(encoder_handle_t handle)
{
    if (!handle)
        return;

    // 更新运行时间
    if (handle->config.sys_tick_get)
    {
        handle->stats.runtime_ticks = handle->config.sys_tick_get() - handle->start_time;
    }
}

static void transition_to_tracking_mode(encoder_handle_t handle, uint32_t found_position)
{
    if (!handle)
        return;

    handle->current_state = ENCODER_STATE_TRACKING;
    handle->current_position.state = ENCODER_STATE_TRACKING;
    handle->current_position.absolute_position = found_position;
    handle->current_position.confidence_level = 100;
    handle->current_position.relative_position_change = 0;

    // 设置位置跟踪器的初始位置
    position_tracker_set_initial_position(handle->position_tracker, (uint16_t)found_position, 1);

    LOGI(absolute_encoder_log_level, "Transitioned to tracking mode at position %d", found_position);
}

static void transition_to_search_mode(encoder_handle_t handle)
{
    if (!handle)
        return;

    handle->current_state = ENCODER_STATE_SEARCHING;
    handle->current_position.state = ENCODER_STATE_SEARCHING;
    handle->current_position.absolute_position = 0;
    handle->current_position.confidence_level = 0;
    handle->current_position.relative_position_change = 0;

    // 重置搜索管理器
    if (handle->search_manager)
    {
        smart_search_manager_reset(handle->search_manager);
    }

    // 重置位置跟踪器
    if (handle->position_tracker)
    {
        position_tracker_reset(handle->position_tracker);
    }

    LOGI(absolute_encoder_log_level, "Transitioned to search mode");
}
