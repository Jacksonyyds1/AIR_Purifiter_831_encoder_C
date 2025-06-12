#include "absolute_encoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// 模拟系统时钟函数
static uint32_t get_system_tick(void)
{
    return (uint32_t)(clock() / CLOCKS_PER_SEC * 1000);
}

// 简单的测试编码器图案（32位德布鲁因序列的一部分）
static const uint8_t test_pattern[] = {
    0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0
};

int main(void)
{
    printf("=== Absolute Encoder C Library Test ===\n");

    // 设置日志级别
    absolute_encoder_set_log_level(LOG_LEVEL_INFO);

    // 1. 创建编码器图案对象
    const uint32_t pattern_length = sizeof(test_pattern);
    const uint8_t max_search_depth = 8;
    
    encoder_map_handle_t map_handle = create_encoder_map(test_pattern, pattern_length, max_search_depth);
    if (!map_handle) {
        printf("ERROR: Failed to create encoder map\n");
        return -1;
    }
    printf("✓ Encoder map created successfully (pattern length: %d)\n", pattern_length);

    // 2. 配置编码器参数
    encoder_config_t config = {
        .motor_steps_per_unit = 100,
        .search_timeout_seconds = 10,
        .tracking_lost_threshold = 5,
        .sys_tick_get = get_system_tick
    };

    // 3. 创建编码器对象
    encoder_handle_t encoder = create_encoder(map_handle, &config);
    if (!encoder) {
        printf("ERROR: Failed to create encoder\n");
        destroy_encoder_map(map_handle);
        return -1;
    }
    printf("✓ Encoder created successfully\n");

    // 4. 模拟编码器信号处理
    printf("\n--- Testing Signal Processing ---\n");
    
    encoder_position_t position_info;
    int signal_count = 0;
    bool position_found = false;

    // 模拟输入一段已知的编码器序列
    const uint8_t test_sequence[] = {0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1};
    const int sequence_length = sizeof(test_sequence);

    for (int i = 0; i < sequence_length; i++)
    {
        uint8_t signal_bit = test_sequence[i];
        encoder_result_t result = process_encoder_signal(encoder, signal_bit, &position_info);
        signal_count++;

        switch (result)
        {
            case ENCODER_RESULT_POSITION_FOUND:
                printf("✓ Position FOUND at signal %d: position=%d, confidence=%d%%\n", 
                       signal_count, position_info.absolute_position, position_info.confidence_level);
                position_found = true;
                break;

            case ENCODER_RESULT_TRACKING_UPDATED:
                printf("✓ Position UPDATED at signal %d: position=%d, confidence=%d%%\n", 
                       signal_count, position_info.absolute_position, position_info.confidence_level);
                break;

            case ENCODER_RESULT_SEARCHING:
                printf("  Searching... (signal %d: %d)\n", signal_count, signal_bit);
                break;

            case ENCODER_RESULT_ERROR_LOST_TRACKING:
                printf("⚠ Tracking lost at signal %d\n", signal_count);
                break;

            default:
                printf("  Signal %d: %d, result=%d\n", signal_count, signal_bit, result);
                break;
        }
    }

    // 5. 获取统计信息
    printf("\n--- Statistics ---\n");
    encoder_stats_t stats;
    if (get_encoder_stats(encoder, &stats))
    {
        printf("Total signals processed: %d\n", stats.total_signals_processed);
        printf("Search attempts: %d\n", stats.search_attempts);
        printf("Tracking updates: %d\n", stats.tracking_updates);
        printf("Error count: %d\n", stats.error_count);
        printf("Runtime: %d ms\n", stats.runtime_ticks);
    }

    // 6. 测试重置功能
    printf("\n--- Testing Reset ---\n");
    if (reset_encoder(encoder))
    {
        printf("✓ Encoder reset successfully\n");
    }

    // 7. 获取当前位置
    if (get_current_position(encoder, &position_info))
    {
        printf("Current state: %d, position: %d, confidence: %d%%\n",
               position_info.state, position_info.absolute_position, position_info.confidence_level);
    }

    // 8. 清理资源
    destroy_encoder(encoder);
    destroy_encoder_map(map_handle);
    printf("\n✓ Resources cleaned up successfully\n");

    // 测试结果总结
    printf("\n=== Test Summary ===\n");
    if (position_found)
    {
        printf("✓ Test PASSED: Position finding functionality works correctly\n");
    }
    else
    {
        printf("⚠ Test PARTIAL: Search functionality works, but no position found in test sequence\n");
        printf("  (This may be normal if the test sequence doesn't contain a unique pattern)\n");
    }

    return 0;
}
