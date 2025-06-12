#ifndef SMART_SEARCH_MANAGER_H
#define SMART_SEARCH_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "encoder_map.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 搜索统计信息
 */
typedef struct
{
    uint32_t total_searches;        // 总搜索次数
    uint32_t successful_searches;   // 成功搜索次数
    uint32_t retry_count;          // 重试次数
    uint32_t max_sequence_length;  // 最大成功序列长度
} search_stats_t;

/**
 * @brief 智能搜索管理器结构
 * 实现搜索失败后的自动重试机制，通过滑动窗口算法提高搜索成功率
 * 专注于搜索模式，不包含跟踪功能
 */
typedef struct smart_search_manager_s
{
    encoder_map_t *encoder_map;         // 编码器映射引用
    uint8_t *buffer;                    // 输入位缓冲区
    uint8_t max_buffer_size;            // 最大缓冲区大小
    uint8_t buffer_size;                // 当前缓冲区大小

    // 当前搜索状态
    uint8_t current_start_offset;       // 当前搜索起始偏移
    tree_node_t *current_search_node;   // 当前搜索节点
    search_direction_t current_direction; // 当前搜索方向
    uint16_t last_processed_index;      // 上次处理的缓冲区索引
    search_stats_t stats;               // 搜索统计信息
} smart_search_manager_t;

/**
 * @brief 创建智能搜索管理器（动态内存版本）
 * @param encoder_map 编码器映射对象指针
 * @param max_buffer_size 最大缓冲区大小
 * @return 智能搜索管理器指针，失败返回NULL
 */
smart_search_manager_t *smart_search_manager_create(encoder_map_t *encoder_map, uint8_t max_buffer_size);

/**
 * @brief 创建智能搜索管理器（静态缓冲区版本）
 * @param encoder_map 编码器映射对象指针
 * @param buffer 预分配的缓冲区
 * @param max_buffer_size 最大缓冲区大小
 * @return 智能搜索管理器指针，失败返回NULL
 */
smart_search_manager_t *smart_search_manager_create_with_buffer(encoder_map_t *encoder_map, uint8_t *buffer, uint8_t max_buffer_size);

/**
 * @brief 销毁智能搜索管理器
 * @param manager 智能搜索管理器指针
 */
void smart_search_manager_destroy(smart_search_manager_t *manager);

/**
 * @brief 添加新的输入位并执行智能搜索
 * @param manager 智能搜索管理器指针
 * @param bit 新输入的位值 (0 或 1)
 * @param direction 搜索方向
 * @param position 输出参数：找到匹配时的位置
 * @return 搜索结果状态
 */
search_result_t smart_search_manager_add_bit_and_search(smart_search_manager_t *manager, uint8_t bit, 
                                                       search_direction_t direction, uint16_t *position);

/**
 * @brief 添加新的输入位并执行智能搜索（带详细信息）
 * @param manager 智能搜索管理器指针
 * @param bit 新输入的位值 (0 或 1)
 * @param direction 搜索方向
 * @param position 输出参数：找到匹配时的位置
 * @param sequence_length 输出参数：匹配序列的长度
 * @param start_offset 输出参数：匹配序列在缓冲区中的起始偏移
 * @return 搜索结果状态
 */
search_result_t smart_search_manager_add_bit_and_search_detailed(smart_search_manager_t *manager, uint8_t bit, 
                                                               search_direction_t direction, uint16_t *position,
                                                               uint8_t *sequence_length, uint8_t *start_offset);

/**
 * @brief 重置搜索状态，清空缓冲区
 * @param manager 智能搜索管理器指针
 */
void smart_search_manager_reset(smart_search_manager_t *manager);

/**
 * @brief 获取当前缓冲区大小
 * @param manager 智能搜索管理器指针
 * @return 当前缓冲区大小
 */
uint8_t smart_search_manager_get_buffer_size(const smart_search_manager_t *manager);

/**
 * @brief 获取当前搜索起始偏移
 * @param manager 智能搜索管理器指针
 * @return 当前搜索起始偏移
 */
uint8_t smart_search_manager_get_current_offset(const smart_search_manager_t *manager);

/**
 * @brief 获取搜索统计信息
 * @param manager 智能搜索管理器指针
 * @param stats 输出参数：统计信息
 */
void smart_search_manager_get_stats(const smart_search_manager_t *manager, search_stats_t *stats);

/**
 * @brief 清零统计信息
 * @param manager 智能搜索管理器指针
 */
void smart_search_manager_clear_stats(smart_search_manager_t *manager);

#ifdef __cplusplus
}
#endif

#endif // SMART_SEARCH_MANAGER_H
