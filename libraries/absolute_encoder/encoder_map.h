#ifndef ENCODER_MAP_H
#define ENCODER_MAP_H

#include <stdint.h>
#include <stdbool.h>
#include "absolute_encoder_logger.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 搜索结果状态
 */
typedef enum
{
    SEARCH_RESULT_FOUND = 0,    // 找到完整匹配
    SEARCH_RESULT_PARTIAL,      // 部分匹配，可继续搜索
    SEARCH_RESULT_NOT_FOUND     // 未找到匹配
} search_result_t;

/**
 * @brief 搜索方向
 */
typedef enum
{
    SEARCH_DIRECTION_FORWARD = 0,   // 正向搜索
    SEARCH_DIRECTION_BACKWARD       // 反向搜索
} search_direction_t;

/**
 * @brief 二叉树节点结构
 */
typedef struct tree_node_s
{
    uint16_t forward_position;      // 正向搜索完成时编码器的当前位置
    uint16_t backward_position;     // 反向搜索完成时编码器的当前位置
    struct tree_node_s *left;      // 0分支
    struct tree_node_s *right;     // 1分支
    struct tree_node_s *parent;    // 父节点
    bool is_forward_leaf;           // 是否为正向搜索的叶子节点
    bool is_backward_leaf;          // 是否为反向搜索的叶子节点
    uint8_t bit_value;              // 当前位的值 (0 或 1)
    uint8_t depth;                  // 节点深度（从根节点开始计数）
} tree_node_t;

/**
 * @brief 编码器映射结构
 */
typedef struct encoder_map_s
{
    uint8_t *pattern;               // 编码器图案数组
    uint16_t pattern_length;        // 图案长度
    uint8_t max_search_depth;       // 最大搜索深度限制
    tree_node_t *root;              // 二叉树根节点
    
    // 内存管理
    bool use_dynamic_memory;        // 是否使用动态内存
    tree_node_t *node_pool;         // 预分配的节点内存池
    uint16_t pool_size;             // 内存池大小（节点数量）
    uint16_t pool_used;             // 已使用的内存池节点数量
    
    // 调试和统计
    uint32_t search_count;          // 搜索次数统计
    uint32_t node_count;            // 节点总数
} encoder_map_t;

/**
 * @brief 创建编码器图案对象（动态内存版本）
 * @param pattern 编码器图案数组
 * @param pattern_length 图案长度
 * @param max_search_depth 最大搜索深度限制
 * @return 编码器图案指针，失败返回NULL
 */
encoder_map_t *encoder_map_create(const uint8_t *pattern, uint16_t pattern_length, uint8_t max_search_depth);

/**
 * @brief 创建编码器图案对象（内存池版本，RTOS适用）
 * @param pattern 编码器图案数组
 * @param pattern_length 图案长度
 * @param max_search_depth 最大搜索深度限制
 * @param node_pool 预分配的节点内存池
 * @param pool_size 内存池大小（节点数量）
 * @return 编码器图案指针，失败返回NULL
 */
encoder_map_t *encoder_map_create_with_pool(const uint8_t *pattern, uint16_t pattern_length, uint8_t max_search_depth,
                                            tree_node_t *node_pool, uint16_t pool_size);

/**
 * @brief 销毁编码器图案对象
 * @param map 编码器图案指针
 */
void encoder_map_destroy(encoder_map_t *map);

/**
 * @brief 开始新的搜索
 * @param map 编码器图案指针
 * @param bit 输入的位值
 * @param direction 搜索方向
 * @return 搜索上下文节点
 */
tree_node_t *encoder_map_start_search(encoder_map_t *map, uint8_t bit, search_direction_t direction);

/**
 * @brief 绝对搜索
 * @param map 编码器图案指针
 * @param current_node 当前搜索上下文（输入输出参数）
 * @param bit 新输入的位值
 * @param direction 搜索方向
 * @param position 输出参数：如果找到匹配，返回位置
 * @return 搜索结果状态
 */
search_result_t encoder_map_absolute_search(encoder_map_t *map, tree_node_t **current_node, uint8_t bit,
                                           search_direction_t direction, uint16_t *position);

/**
 * @brief 获取指定位置的位值
 * @param map 编码器图案指针
 * @param position 位置索引
 * @return 位值 (0 或 1)
 */
uint8_t encoder_map_get_bit_at_position(const encoder_map_t *map, uint16_t position);

/**
 * @brief 获取图案长度
 * @param map 编码器图案指针
 * @return 图案长度
 */
uint16_t encoder_map_get_pattern_length(const encoder_map_t *map);

/**
 * @brief 获取统计信息
 * @param map 编码器图案指针
 * @param search_count 输出参数：搜索次数
 * @param node_count 输出参数：节点总数
 */
void encoder_map_get_stats(const encoder_map_t *map, uint32_t *search_count, uint32_t *node_count);

/**
 * @brief 重置统计信息
 * @param map 编码器图案指针
 */
void encoder_map_reset_stats(encoder_map_t *map);

/**
 * @brief 打印调试信息
 * @param map 编码器图案指针
 * @param log_level 日志级别
 */
void encoder_map_print_debug_info(const encoder_map_t *map, int log_level);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_MAP_H
