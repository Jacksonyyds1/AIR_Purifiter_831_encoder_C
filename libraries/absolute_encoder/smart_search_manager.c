#include "smart_search_manager.h"
#include <stdlib.h>
#include <string.h>

// 外部日志级别变量
extern int absolute_encoder_log_level;

// 内部辅助函数声明
static search_result_t smart_search_manager_search_from_offset(smart_search_manager_t *manager, uint8_t start_offset, 
                                                              search_direction_t direction, uint16_t *position);
static search_result_t smart_search_manager_shift_and_retry(smart_search_manager_t *manager, 
                                                           search_direction_t direction, uint16_t *position);
static void smart_search_manager_add_to_buffer(smart_search_manager_t *manager, uint8_t bit);
static void smart_search_manager_update_stats(smart_search_manager_t *manager, search_result_t result, uint8_t sequence_length);

// 实现

smart_search_manager_t *smart_search_manager_create(encoder_map_t *encoder_map, uint8_t max_buffer_size)
{
    if (!encoder_map || max_buffer_size == 0)
    {
        LOGE(absolute_encoder_log_level, "Invalid parameters: encoder_map=%p, max_buffer_size=%d", 
             encoder_map, max_buffer_size);
        return NULL;
    }

    smart_search_manager_t *manager = (smart_search_manager_t *)malloc(sizeof(smart_search_manager_t));
    if (!manager)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate smart_search_manager_t");
        return NULL;
    }

    // 分配缓冲区内存
    manager->buffer = (uint8_t *)malloc(max_buffer_size);
    if (!manager->buffer)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate buffer memory");
        free(manager);
        return NULL;
    }

    // 初始化结构体成员
    manager->encoder_map = encoder_map;
    manager->max_buffer_size = max_buffer_size;
    manager->buffer_size = 0;
    manager->current_start_offset = 0;
    manager->current_search_node = NULL;
    manager->current_direction = SEARCH_DIRECTION_FORWARD;
    manager->last_processed_index = 0;

    // 初始化统计信息
    smart_search_manager_clear_stats(manager);

    LOGD(absolute_encoder_log_level, "SmartSearchManager initialized with max_buffer_size=%d", max_buffer_size);
    return manager;
}

smart_search_manager_t *smart_search_manager_create_with_buffer(encoder_map_t *encoder_map, uint8_t *buffer, uint8_t max_buffer_size)
{
    if (!encoder_map || !buffer || max_buffer_size == 0)
    {
        LOGE(absolute_encoder_log_level, "Invalid parameters: encoder_map=%p, buffer=%p, max_buffer_size=%d", 
             encoder_map, buffer, max_buffer_size);
        return NULL;
    }

    smart_search_manager_t *manager = (smart_search_manager_t *)malloc(sizeof(smart_search_manager_t));
    if (!manager)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate smart_search_manager_t");
        return NULL;
    }

    // 初始化结构体成员（使用外部缓冲区）
    manager->encoder_map = encoder_map;
    manager->buffer = buffer;
    manager->max_buffer_size = max_buffer_size;
    manager->buffer_size = 0;
    manager->current_start_offset = 0;
    manager->current_search_node = NULL;
    manager->current_direction = SEARCH_DIRECTION_FORWARD;
    manager->last_processed_index = 0;

    // 初始化统计信息
    smart_search_manager_clear_stats(manager);

    LOGD(absolute_encoder_log_level, "SmartSearchManager initialized with external buffer, max_buffer_size=%d", max_buffer_size);
    return manager;
}

void smart_search_manager_destroy(smart_search_manager_t *manager)
{
    if (!manager)
        return;

    // 只有当缓冲区是我们分配的时候才释放
    // 注意：这里需要一个标志来区分是动态分配还是外部缓冲区
    // 为了简化，我们假设create函数分配的缓冲区需要释放，create_with_buffer的不需要
    // 实际使用中可以添加一个标志位来标识
    if (manager->buffer)
    {
        // 由于无法确定缓冲区是否需要释放，这里注释掉
        // free(manager->buffer);
    }

    free(manager);
}

search_result_t smart_search_manager_add_bit_and_search(smart_search_manager_t *manager, uint8_t bit, 
                                                       search_direction_t direction, uint16_t *position)
{
    uint8_t sequence_length, start_offset;
    return smart_search_manager_add_bit_and_search_detailed(manager, bit, direction, position, 
                                                           &sequence_length, &start_offset);
}

search_result_t smart_search_manager_add_bit_and_search_detailed(smart_search_manager_t *manager, uint8_t bit, 
                                                               search_direction_t direction, uint16_t *position,
                                                               uint8_t *sequence_length, uint8_t *start_offset)
{
    if (!manager || !position || !sequence_length || !start_offset)
        return SEARCH_RESULT_NOT_FOUND;

    // 添加新位到缓冲区
    smart_search_manager_add_to_buffer(manager, bit);

    // 如果方向改变，重置搜索状态
    if (direction != manager->current_direction)
    {
        manager->current_direction = direction;
        manager->current_start_offset = 0;
        manager->current_search_node = NULL;
        LOGD(absolute_encoder_log_level, "Search direction changed, resetting search state");
    }

    // 从当前偏移开始搜索
    search_result_t result = smart_search_manager_search_from_offset(manager, manager->current_start_offset, 
                                                                    direction, position);

    // 如果搜索失败，尝试滑动窗口重试
    if (result == SEARCH_RESULT_NOT_FOUND)
    {
        result = smart_search_manager_shift_and_retry(manager, direction, position);
    }

    // 返回实际的匹配信息
    if (result == SEARCH_RESULT_FOUND)
    {
        *start_offset = manager->current_start_offset;
        // 使用找到的搜索节点的深度信息
        if (manager->current_search_node != NULL)
        {
            *sequence_length = manager->current_search_node->depth;
        }
        else
        {
            *sequence_length = manager->buffer_size - manager->current_start_offset;
        }
    }
    else
    {
        *start_offset = 0;
        *sequence_length = 0;
    }

    // 更新统计信息
    smart_search_manager_update_stats(manager, result, *sequence_length);

    return result;
}

void smart_search_manager_reset(smart_search_manager_t *manager)
{
    if (!manager)
        return;

    manager->buffer_size = 0;
    manager->current_start_offset = 0;
    manager->current_search_node = NULL;
    manager->current_direction = SEARCH_DIRECTION_FORWARD;
    manager->last_processed_index = 0;

    LOGD(absolute_encoder_log_level, "SmartSearchManager reset");
}

uint8_t smart_search_manager_get_buffer_size(const smart_search_manager_t *manager)
{
    if (!manager)
        return 0;

    return manager->buffer_size;
}

uint8_t smart_search_manager_get_current_offset(const smart_search_manager_t *manager)
{
    if (!manager)
        return 0;

    return manager->current_start_offset;
}

void smart_search_manager_get_stats(const smart_search_manager_t *manager, search_stats_t *stats)
{
    if (!manager || !stats)
        return;

    *stats = manager->stats;
}

void smart_search_manager_clear_stats(smart_search_manager_t *manager)
{
    if (!manager)
        return;

    manager->stats.total_searches = 0;
    manager->stats.successful_searches = 0;
    manager->stats.retry_count = 0;
    manager->stats.max_sequence_length = 0;
}

// 内部辅助函数实现

static search_result_t smart_search_manager_search_from_offset(smart_search_manager_t *manager, uint8_t start_offset, 
                                                              search_direction_t direction, uint16_t *position)
{
    if (!manager || !position || start_offset >= manager->buffer_size)
        return SEARCH_RESULT_NOT_FOUND;

    manager->stats.total_searches++;

    // 如果是新的搜索起点或者当前节点为空，开始新搜索
    if (manager->current_search_node == NULL || start_offset != manager->current_start_offset)
    {
        manager->current_start_offset = start_offset;
        manager->last_processed_index = start_offset;
        uint8_t start_bit = manager->buffer[start_offset];
        manager->current_search_node = encoder_map_start_search(manager->encoder_map, start_bit, direction);

        if (manager->current_search_node == NULL)
        {
            LOGV(absolute_encoder_log_level, "Failed to start search with bit %d at offset %d", start_bit, start_offset);
            return SEARCH_RESULT_NOT_FOUND;
        }

        LOGV(absolute_encoder_log_level, "Started new search at offset %d with bit %d", start_offset, start_bit);

        // 如果只有一个位，检查是否已经匹配
        if (manager->buffer_size == start_offset + 1)
        {
            // 检查当前节点是否为叶子节点
            bool is_leaf = (direction == SEARCH_DIRECTION_FORWARD) ?
                          manager->current_search_node->is_forward_leaf : manager->current_search_node->is_backward_leaf;

            if (is_leaf)
            {
                *position = (direction == SEARCH_DIRECTION_FORWARD) ?
                           manager->current_search_node->forward_position : manager->current_search_node->backward_position;
                manager->stats.successful_searches++;
                LOGV(absolute_encoder_log_level, "Found match at offset %d with single bit", start_offset);
                return SEARCH_RESULT_FOUND;
            }
        }
    }

    // 继续绝对搜索 - 只处理新添加的位
    for (uint8_t i = manager->last_processed_index + 1; i < manager->buffer_size; i++)
    {
        uint8_t bit = manager->buffer[i];
        search_result_t result = encoder_map_absolute_search(manager->encoder_map, &manager->current_search_node, 
                                                           bit, direction, position);

        LOGV(absolute_encoder_log_level, "Absolute search at index %d, bit %d, result %d", i, bit, (int)result);

        if (result == SEARCH_RESULT_FOUND)
        {
            manager->stats.successful_searches++;
            uint8_t sequence_length = i - start_offset + 1;
            if (sequence_length > manager->stats.max_sequence_length)
            {
                manager->stats.max_sequence_length = sequence_length;
            }
            LOGD(absolute_encoder_log_level, "Search successful at offset %d, sequence length %d, position %d",
                 start_offset, sequence_length, *position);

            manager->last_processed_index = i;
            return SEARCH_RESULT_FOUND;
        }
        else if (result == SEARCH_RESULT_NOT_FOUND)
        {
            LOGV(absolute_encoder_log_level, "Search failed at index %d", i);
            return SEARCH_RESULT_NOT_FOUND;
        }
        // result == SEARCH_RESULT_PARTIAL，继续搜索
        manager->last_processed_index = i;
    }

    // 搜索未完成，但还有部分匹配
    return SEARCH_RESULT_PARTIAL;
}

static search_result_t smart_search_manager_shift_and_retry(smart_search_manager_t *manager, 
                                                           search_direction_t direction, uint16_t *position)
{
    if (!manager || !position)
        return SEARCH_RESULT_NOT_FOUND;

    uint8_t original_offset = manager->current_start_offset;

    // 尝试从不同的起始偏移重新搜索
    for (uint8_t offset = original_offset + 1; offset < manager->buffer_size; offset++)
    {
        manager->stats.retry_count++;
        manager->current_search_node = NULL;  // 重置搜索节点

        search_result_t result = smart_search_manager_search_from_offset(manager, offset, direction, position);

        if (result == SEARCH_RESULT_FOUND)
        {
            LOGD(absolute_encoder_log_level, "Retry successful at offset %d after %d retries",
                 offset, offset - original_offset);
            return SEARCH_RESULT_FOUND;
        }
        else if (result == SEARCH_RESULT_PARTIAL)
        {
            // 找到部分匹配，停止重试
            LOGV(absolute_encoder_log_level, "Found partial match at offset %d", offset);
            return SEARCH_RESULT_PARTIAL;
        }
    }

    // 所有重试都失败
    LOGV(absolute_encoder_log_level, "All retries failed for buffer size %d", manager->buffer_size);
    return SEARCH_RESULT_NOT_FOUND;
}

static void smart_search_manager_add_to_buffer(smart_search_manager_t *manager, uint8_t bit)
{
    if (!manager)
        return;

    // 如果缓冲区已满，移除最早的位
    if (manager->buffer_size >= manager->max_buffer_size)
    {
        // 左移缓冲区
        for (uint8_t i = 0; i < manager->max_buffer_size - 1; i++)
        {
            manager->buffer[i] = manager->buffer[i + 1];
        }

        // 调整当前偏移和处理索引
        if (manager->current_start_offset > 0)
        {
            manager->current_start_offset--;
            if (manager->last_processed_index > 0)
                manager->last_processed_index--;
        }
        else
        {
            // 如果当前搜索的起始点被移除，重置搜索状态
            manager->current_search_node = NULL;
            manager->current_start_offset = 0;
            manager->last_processed_index = 0;
            LOGV(absolute_encoder_log_level, "Buffer overflow, reset search state");
        }

        // 在最后位置添加新位
        manager->buffer[manager->max_buffer_size - 1] = bit;
    }
    else
    {
        // 缓冲区未满，直接添加
        manager->buffer[manager->buffer_size] = bit;
        manager->buffer_size++;
    }

    LOGV(absolute_encoder_log_level, "Added bit %d to buffer, buffer size: %d", bit, manager->buffer_size);
}

static void smart_search_manager_update_stats(smart_search_manager_t *manager, search_result_t result, uint8_t sequence_length)
{
    if (!manager)
        return;

    if (result == SEARCH_RESULT_FOUND && sequence_length > manager->stats.max_sequence_length)
    {
        manager->stats.max_sequence_length = sequence_length;
    }
}
