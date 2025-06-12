#include "encoder_map.h"
#include <stdlib.h>
#include <string.h>

// 外部日志级别变量
extern int absolute_encoder_log_level;

// 内部辅助函数声明
static void encoder_map_build_search_tree(encoder_map_t *map);
static void encoder_map_build_forward_sequences(encoder_map_t *map);
static void encoder_map_build_backward_sequences(encoder_map_t *map);
static bool encoder_map_is_subsequence_unique(const encoder_map_t *map, uint16_t start_pos, uint8_t length, search_direction_t direction);
static bool encoder_map_has_shorter_unique_subsequence(const encoder_map_t *map, uint16_t start_pos, uint8_t length, search_direction_t direction);
static void encoder_map_insert_unique_subsequence(encoder_map_t *map, uint16_t start_pos, uint8_t length, search_direction_t direction);
static tree_node_t *encoder_map_allocate_node(encoder_map_t *map);
static void encoder_map_destroy_tree(encoder_map_t *map, tree_node_t *node);

// 实现

encoder_map_t *encoder_map_create(const uint8_t *pattern, uint16_t pattern_length, uint8_t max_search_depth)
{
    if (!pattern || pattern_length == 0 || max_search_depth == 0)
    {
        LOGE(absolute_encoder_log_level, "Invalid parameters: pattern=%p, length=%d, max_depth=%d",
             pattern, pattern_length, max_search_depth);
        return NULL;
    }

    encoder_map_t *map = (encoder_map_t *)malloc(sizeof(encoder_map_t));
    if (!map)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate encoder_map_t");
        return NULL;
    }

    // 分配图案数据内存并复制
    map->pattern = (uint8_t *)malloc(pattern_length);
    if (!map->pattern)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate pattern memory");
        free(map);
        return NULL;
    }
    memcpy(map->pattern, pattern, pattern_length);

    // 初始化结构体成员
    map->pattern_length = pattern_length;
    map->max_search_depth = max_search_depth;
    map->root = NULL;
    map->use_dynamic_memory = true;
    map->node_pool = NULL;
    map->pool_size = 0;
    map->pool_used = 0;
    map->search_count = 0;
    map->node_count = 0;

    LOGD(absolute_encoder_log_level, "EncoderMap create: pattern_length=%d, max_search_depth=%d",
         pattern_length, max_search_depth);

    encoder_map_build_search_tree(map);
    return map;
}

encoder_map_t *encoder_map_create_with_pool(const uint8_t *pattern, uint16_t pattern_length, uint8_t max_search_depth,
                                            tree_node_t *node_pool, uint16_t pool_size)
{
    if (!pattern || pattern_length == 0 || max_search_depth == 0 || !node_pool || pool_size == 0)
    {
        LOGE(absolute_encoder_log_level, "Invalid pool parameters: pattern=%p, length=%d, max_depth=%d, pool=%p, pool_size=%d",
             pattern, pattern_length, max_search_depth, node_pool, pool_size);
        return NULL;
    }

    encoder_map_t *map = (encoder_map_t *)malloc(sizeof(encoder_map_t));
    if (!map)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate encoder_map_t");
        return NULL;
    }

    // 分配图案数据内存并复制
    map->pattern = (uint8_t *)malloc(pattern_length);
    if (!map->pattern)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate pattern memory");
        free(map);
        return NULL;
    }
    memcpy(map->pattern, pattern, pattern_length);

    // 初始化结构体成员
    map->pattern_length = pattern_length;
    map->max_search_depth = max_search_depth;
    map->root = NULL;
    map->use_dynamic_memory = false;
    map->node_pool = node_pool;
    map->pool_size = pool_size;
    map->pool_used = 0;
    map->search_count = 0;
    map->node_count = 0;

    LOGD(absolute_encoder_log_level, "EncoderMap create with pool: pattern_length=%d, max_search_depth=%d, pool_size=%d",
         pattern_length, max_search_depth, pool_size);

    // 初始化内存池
    for (uint16_t i = 0; i < pool_size; i++)
    {
        memset(&node_pool[i], 0, sizeof(tree_node_t));
    }

    encoder_map_build_search_tree(map);
    return map;
}

void encoder_map_destroy(encoder_map_t *map)
{
    if (!map)
        return;

    if (map->use_dynamic_memory)
    {
        // 动态分配模式才需要释放内存
        encoder_map_destroy_tree(map, map->root);
    }
    // 内存池模式不需要释放，由外部管理

    if (map->pattern)
    {
        free(map->pattern);
    }

    free(map);
}

tree_node_t *encoder_map_start_search(encoder_map_t *map, uint8_t bit, search_direction_t direction)
{
    if (!map || !map->root)
        return NULL;

    // 避免未使用参数警告
    (void)direction;

    map->search_count++;

    // 根据输入位选择分支
    if (bit == 0)
    {
        return map->root->left;
    }
    else
    {
        return map->root->right;
    }
}

search_result_t encoder_map_absolute_search(encoder_map_t *map, tree_node_t **current_node, uint8_t bit,
                                           search_direction_t direction, uint16_t *position)
{
    if (!map || !current_node || !*current_node || !position)
        return SEARCH_RESULT_NOT_FOUND;

    tree_node_t *node = *current_node;

    // 根据输入位选择下一个节点
    tree_node_t *next_node = (bit == 0) ? node->left : node->right;

    if (!next_node)
    {
        // 没有匹配的路径
        return SEARCH_RESULT_NOT_FOUND;
    }

    // 更新当前节点
    *current_node = next_node;

    // 检查是否到达叶子节点
    bool is_leaf = false;
    if (direction == SEARCH_DIRECTION_FORWARD && next_node->is_forward_leaf)
    {
        *position = next_node->forward_position;
        is_leaf = true;
    }
    else if (direction == SEARCH_DIRECTION_BACKWARD && next_node->is_backward_leaf)
    {
        *position = next_node->backward_position;
        is_leaf = true;
    }

    if (is_leaf)
    {
        return SEARCH_RESULT_FOUND;
    }
    else
    {
        return SEARCH_RESULT_PARTIAL;
    }
}

uint8_t encoder_map_get_bit_at_position(const encoder_map_t *map, uint16_t position)
{
    if (!map || position >= map->pattern_length)
        return 0;

    return map->pattern[position];
}

uint16_t encoder_map_get_pattern_length(const encoder_map_t *map)
{
    if (!map)
        return 0;

    return map->pattern_length;
}

void encoder_map_get_stats(const encoder_map_t *map, uint32_t *search_count, uint32_t *node_count)
{
    if (!map)
        return;

    if (search_count)
        *search_count = map->search_count;
    if (node_count)
        *node_count = map->node_count;
}

void encoder_map_reset_stats(encoder_map_t *map)
{
    if (!map)
        return;

    map->search_count = 0;
}

void encoder_map_print_debug_info(const encoder_map_t *map, int log_level)
{
    if (!map)
        return;

    LOGI(log_level, "EncoderMap Debug Info:");
    LOGI(log_level, "  Pattern length: %d", map->pattern_length);
    LOGI(log_level, "  Max search depth: %d", map->max_search_depth);
    LOGI(log_level, "  Use dynamic memory: %s", map->use_dynamic_memory ? "true" : "false");
    LOGI(log_level, "  Node count: %d", map->node_count);
    LOGI(log_level, "  Search count: %d", map->search_count);
    if (!map->use_dynamic_memory)
    {
        LOGI(log_level, "  Pool size: %d", map->pool_size);
        LOGI(log_level, "  Pool used: %d", map->pool_used);
    }
}

// 内部辅助函数实现

static void encoder_map_build_search_tree(encoder_map_t *map)
{
    if (!map)
        return;

    // 创建根节点
    map->root = encoder_map_allocate_node(map);
    if (!map->root)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate root node");
        return;
    }

    map->root->depth = 0;

    LOGD(absolute_encoder_log_level, "Starting to build search tree");
    // 先构建正向序列，再在同一棵树上添加反向信息
    encoder_map_build_forward_sequences(map);
    encoder_map_build_backward_sequences(map);
    LOGD(absolute_encoder_log_level, "Search tree construction completed");
}

static void encoder_map_build_forward_sequences(encoder_map_t *map)
{
    if (!map)
        return;

    LOGD(absolute_encoder_log_level, "Building forward sequences");

    // 记录已经被添加到树中的位置
    bool *position_covered = (bool *)calloc(map->pattern_length, sizeof(bool));
    if (!position_covered)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate position_covered array");
        return;
    }

    uint16_t covered_count = 0;

    // 从长度1开始，逐步增加长度寻找唯一子图
    for (uint8_t length = 1; length <= map->max_search_depth && covered_count < map->pattern_length; length++)
    {
        LOGV(absolute_encoder_log_level, "Checking subsequences of length %d", length);

        uint16_t added_this_length = 0;

        // 检查每个位置的子图
        for (uint16_t start_pos = 0; start_pos < map->pattern_length; start_pos++)
        {
            // 计算该起始位置对应的目标位置
            uint16_t target_pos = (start_pos + length - 1) % map->pattern_length;

            // 如果目标位置已经被更短的子图覆盖，跳过
            if (position_covered[target_pos])
            {
                continue;
            }

            // 检查该子图是否唯一
            if (encoder_map_is_subsequence_unique(map, start_pos, length, SEARCH_DIRECTION_FORWARD))
            {
                // 检查是否有更短的唯一子图是其子图
                if (!encoder_map_has_shorter_unique_subsequence(map, start_pos, length, SEARCH_DIRECTION_FORWARD))
                {
                    // 插入到搜索树
                    encoder_map_insert_unique_subsequence(map, start_pos, length, SEARCH_DIRECTION_FORWARD);

                    // 标记目标位置（搜索完成后编码器的位置）而不是起始位置
                    if (!position_covered[target_pos])
                    {
                        position_covered[target_pos] = true;
                        covered_count++;
                    }
                    added_this_length++;

                    LOGV(absolute_encoder_log_level, "Added forward subsequence: start=%d, length=%d, target_pos=%d", 
                         start_pos, length, target_pos);
                }
            }
        }

        LOGV(absolute_encoder_log_level, "Added %d subsequences of length %d", added_this_length, length);

        // 如果某个长度没有添加任何子图，说明所有位置都被更短的子图覆盖
        if (added_this_length == 0 && covered_count == map->pattern_length)
        {
            LOGD(absolute_encoder_log_level, "All positions covered by shorter subsequences, stopping at length %d", length);
            break;
        }
    }

    LOGD(absolute_encoder_log_level, "Forward tree construction completed, covered %d/%d positions", 
         covered_count, map->pattern_length);

    free(position_covered);
}

static void encoder_map_build_backward_sequences(encoder_map_t *map)
{
    if (!map)
        return;

    LOGD(absolute_encoder_log_level, "Building backward sequences");

    // 记录已经被添加到树中的位置
    bool *position_covered = (bool *)calloc(map->pattern_length, sizeof(bool));
    if (!position_covered)
    {
        LOGE(absolute_encoder_log_level, "Failed to allocate position_covered array");
        return;
    }

    uint16_t covered_count = 0;

    // 从长度1开始，逐步增加长度寻找唯一子图
    for (uint8_t length = 1; length <= map->max_search_depth && covered_count < map->pattern_length; length++)
    {
        LOGV(absolute_encoder_log_level, "Checking backward subsequences of length %d", length);

        uint16_t added_this_length = 0;

        // 检查每个位置的反向子图
        for (uint16_t start_pos = 0; start_pos < map->pattern_length; start_pos++)
        {
            // 计算该起始位置对应的目标位置（反向）
            uint16_t target_pos = (start_pos - length + 1 + map->pattern_length) % map->pattern_length;

            // 如果目标位置已经被更短的子图覆盖，跳过
            if (position_covered[target_pos])
            {
                continue;
            }

            // 检查该反向子图是否唯一
            if (encoder_map_is_subsequence_unique(map, start_pos, length, SEARCH_DIRECTION_BACKWARD))
            {
                // 检查是否有更短的唯一子图是其子图
                if (!encoder_map_has_shorter_unique_subsequence(map, start_pos, length, SEARCH_DIRECTION_BACKWARD))
                {
                    // 插入到搜索树
                    encoder_map_insert_unique_subsequence(map, start_pos, length, SEARCH_DIRECTION_BACKWARD);

                    // 标记目标位置（反向搜索完成后编码器的位置）
                    if (!position_covered[target_pos])
                    {
                        position_covered[target_pos] = true;
                        covered_count++;
                    }
                    added_this_length++;

                    LOGV(absolute_encoder_log_level, "Added backward subsequence: start=%d, length=%d, target_pos=%d", 
                         start_pos, length, target_pos);
                }
            }
        }

        LOGV(absolute_encoder_log_level, "Added %d backward subsequences of length %d", added_this_length, length);

        // 如果某个长度没有添加任何子图，说明所有位置都被更短的子图覆盖
        if (added_this_length == 0 && covered_count == map->pattern_length)
        {
            LOGD(absolute_encoder_log_level, "All positions covered by shorter subsequences, stopping at length %d", length);
            break;
        }
    }

    LOGD(absolute_encoder_log_level, "Backward tree construction completed, covered %d/%d positions", 
         covered_count, map->pattern_length);

    free(position_covered);
}

static bool encoder_map_is_subsequence_unique(const encoder_map_t *map, uint16_t start_pos, uint8_t length, search_direction_t direction)
{
    if (!map)
        return false;

    // 检查从start_pos开始长度为length的子图是否在码盘上唯一
    for (uint16_t other_pos = 0; other_pos < map->pattern_length; other_pos++)
    {
        if (other_pos == start_pos)
        {
            continue;
        }

        bool matches = true;
        for (uint8_t i = 0; i < length; i++)
        {
            uint16_t pos1, pos2;

            if (direction == SEARCH_DIRECTION_FORWARD)
            {
                pos1 = (start_pos + i) % map->pattern_length;
                pos2 = (other_pos + i) % map->pattern_length;
            }
            else
            {
                pos1 = (start_pos - i + map->pattern_length) % map->pattern_length;
                pos2 = (other_pos - i + map->pattern_length) % map->pattern_length;
            }

            if (map->pattern[pos1] != map->pattern[pos2])
            {
                matches = false;
                break;
            }
        }

        if (matches)
        {
            return false; // 找到重复，不唯一
        }
    }

    return true; // 唯一
}

static bool encoder_map_has_shorter_unique_subsequence(const encoder_map_t *map, uint16_t start_pos, uint8_t length, search_direction_t direction)
{
    if (!map)
        return false;

    // 检查是否存在更短的唯一子图，且该短子图的整个序列处于当前序列的范围内
    for (uint8_t shorter_len = 1; shorter_len < length; shorter_len++)
    {
        // 检查所有可能的子图位置
        for (uint8_t offset = 0; offset <= length - shorter_len; offset++)
        {
            uint16_t sub_start;

            if (direction == SEARCH_DIRECTION_FORWARD)
            {
                sub_start = (start_pos + offset) % map->pattern_length;
            }
            else
            {
                sub_start = (start_pos - offset + map->pattern_length) % map->pattern_length;
            }

            // 检查短子图是否唯一
            if (encoder_map_is_subsequence_unique(map, sub_start, shorter_len, direction))
            {
                // 只需要检查短子图的首尾位置是否都在当前序列的首尾位置区间内
                uint16_t sub_end;
                uint16_t current_start = start_pos;
                uint16_t current_end;

                if (direction == SEARCH_DIRECTION_FORWARD)
                {
                    sub_end = (sub_start + shorter_len - 1) % map->pattern_length;
                    current_end = (start_pos + length - 1) % map->pattern_length;
                }
                else
                {
                    sub_end = (sub_start - shorter_len + 1 + map->pattern_length) % map->pattern_length;
                    current_end = (start_pos - length + 1 + map->pattern_length) % map->pattern_length;
                }

                // 检查短子图的首尾位置是否都在当前序列的区间内
                bool is_contained = false;

                if (direction == SEARCH_DIRECTION_FORWARD)
                {
                    // 正向：检查 [sub_start, sub_end] 是否在 [current_start, current_end] 内
                    if (current_start <= current_end)
                    {
                        // 非跨越边界情况
                        is_contained = (sub_start >= current_start && sub_end <= current_end);
                    }
                    else
                    {
                        // 跨越边界情况
                        is_contained = ((sub_start >= current_start || sub_start <= current_end) &&
                                       (sub_end >= current_start || sub_end <= current_end));
                    }
                }
                else
                {
                    // 反向：检查 [sub_end, sub_start] 是否在 [current_end, current_start] 内
                    if (current_end <= current_start)
                    {
                        // 非跨越边界情况
                        is_contained = (sub_end >= current_end && sub_start <= current_start);
                    }
                    else
                    {
                        // 跨越边界情况
                        is_contained = ((sub_end >= current_end || sub_end <= current_start) &&
                                       (sub_start >= current_end || sub_start <= current_start));
                    }
                }

                if (is_contained)
                {
                    return true; // 找到更短的唯一子图且完全包含在当前序列中
                }
            }
        }
    }

    return false; // 没有找到包含的更短唯一子图
}

static void encoder_map_insert_unique_subsequence(encoder_map_t *map, uint16_t start_pos, uint8_t length, search_direction_t direction)
{
    if (!map || !map->root)
        return;

    tree_node_t *current = map->root;

    // 沿着子图的位值路径在树中创建节点
    for (uint8_t i = 0; i < length; i++)
    {
        uint16_t bit_pos;

        if (direction == SEARCH_DIRECTION_FORWARD)
        {
            bit_pos = (start_pos + i) % map->pattern_length;
        }
        else
        {
            bit_pos = (start_pos - i + map->pattern_length) % map->pattern_length;
        }

        uint8_t bit = map->pattern[bit_pos];
        tree_node_t **next_node = (bit == 0) ? &(current->left) : &(current->right);

        if (*next_node == NULL)
        {
            *next_node = encoder_map_allocate_node(map);
            if (*next_node == NULL)
            {
                LOGE(absolute_encoder_log_level, "Failed to allocate node for subsequence insertion");
                return;
            }
            (*next_node)->bit_value = bit;
            (*next_node)->parent = current;
            (*next_node)->depth = i + 1;
        }

        current = *next_node;
    }

    // 在最后一个节点设置叶子信息
    if (direction == SEARCH_DIRECTION_FORWARD)
    {
        current->is_forward_leaf = true;
        // 正向搜索：当前位置 = 起始位置 + 子图长度 - 1
        current->forward_position = (start_pos + length - 1) % map->pattern_length;
    }
    else
    {
        current->is_backward_leaf = true;
        // 反向搜索：当前位置 = 起始位置 - 子图长度 + 1
        current->backward_position = (start_pos - length + 1 + map->pattern_length) % map->pattern_length;
    }

    LOGV(absolute_encoder_log_level, "Inserted %s subsequence: start=%d, length=%d, end_pos=%d",
         direction == SEARCH_DIRECTION_FORWARD ? "forward" : "backward",
         start_pos, length,
         direction == SEARCH_DIRECTION_FORWARD ? current->forward_position : current->backward_position);
}

static tree_node_t *encoder_map_allocate_node(encoder_map_t *map)
{
    if (!map)
        return NULL;

    tree_node_t *node = NULL;

    if (map->use_dynamic_memory)
    {
        // 动态内存分配
        node = (tree_node_t *)malloc(sizeof(tree_node_t));
        if (node)
        {
            memset(node, 0, sizeof(tree_node_t));
            map->node_count++;
        }
    }
    else
    {
        // 内存池分配
        if (map->pool_used < map->pool_size)
        {
            node = &map->node_pool[map->pool_used];
            memset(node, 0, sizeof(tree_node_t));
            map->pool_used++;
            map->node_count++;
        }
    }

    return node;
}

static void encoder_map_destroy_tree(encoder_map_t *map, tree_node_t *node)
{
    if (!map || !node)
        return;

    // 递归销毁子树
    if (node->left)
        encoder_map_destroy_tree(map, node->left);
    if (node->right)
        encoder_map_destroy_tree(map, node->right);

    // 只有动态内存模式才需要释放节点
    if (map->use_dynamic_memory)
    {
        free(node);
    }
}
