# Absolute Encoder C Library

这是绝对编码器库的C语言版本，从原始的C++代码完整转写而来，保持了所有原有功能。

## 功能特性

- **编码器图案映射** (encoder_map): 二叉树搜索算法，支持正向和反向搜索
- **智能搜索管理器** (smart_search_manager): 滑动窗口搜索，自动重试机制
- **位置跟踪器** (position_tracker): 实时位置跟踪，置信度管理，自动恢复
- **输入预处理器** (input_preprocessor): 电机步数换算，防抖滤波，自适应修正
- **日志系统** (absolute_encoder_logger): 分级日志，可配置开关

## 文件结构

```
absolute_encoder_c/
├── absolute_encoder.h           # 主接口头文件
├── absolute_encoder.c           # 主接口实现
├── encoder_map.h               # 编码器图案映射头文件
├── encoder_map.c               # 编码器图案映射实现
├── smart_search_manager.h      # 智能搜索管理器头文件
├── smart_search_manager.c      # 智能搜索管理器实现
├── position_tracker.h          # 位置跟踪器头文件
├── position_tracker.c          # 位置跟踪器实现
├── input_preprocessor.h        # 输入预处理器头文件
├── input_preprocessor.c        # 输入预处理器实现
├── absolute_encoder_logger.h   # 日志系统头文件
├── Makefile                    # 编译脚本
└── README.md                   # 本文件
```

## 编译方法

### 使用Makefile编译

```bash
# 编译库
make

# 清理编译文件
make clean

# 语法检查
make check

# 查看帮助
make help
```

### 手动编译

```bash
# 编译所有源文件
gcc -Wall -Wextra -Werror -std=c99 -O2 -c *.c

# 创建静态库
ar rcs libabsolute_encoder_c.a *.o
```

## 使用方法

### 1. 基本使用流程

```c
#include "absolute_encoder.h"

// 1. 定义编码器图案数据
static const uint8_t pattern_data[] = {0, 1, 1, 0, 1, 0, 0, 1, ...};
static const uint32_t pattern_length = sizeof(pattern_data);

// 2. 创建编码器图案对象
encoder_map_handle_t map_handle = create_encoder_map(pattern_data, pattern_length, 8);
if (!map_handle) {
    // 错误处理
    return -1;
}

// 3. 配置编码器参数
encoder_config_t config = {
    .motor_steps_per_unit = 100,    // 电机100步对应编码器1个单位
    .search_timeout_seconds = 10,   // 搜索超时10秒
    .tracking_lost_threshold = 5,   // 跟踪丢失阈值
    .sys_tick_get = get_system_tick // 系统时钟函数
};

// 4. 创建编码器对象
encoder_handle_t encoder = create_encoder(map_handle, &config);
if (!encoder) {
    destroy_encoder_map(map_handle);
    return -1;
}

// 5. 处理编码器信号
encoder_position_t position_info;
uint8_t signal_bit = read_encoder_signal(); // 读取硬件信号

encoder_result_t result = process_encoder_signal(encoder, signal_bit, &position_info);

switch (result) {
    case ENCODER_RESULT_POSITION_FOUND:
        printf("Position found: %d\n", position_info.absolute_position);
        break;
    case ENCODER_RESULT_TRACKING_UPDATED:
        printf("Position updated: %d, confidence: %d\n", 
               position_info.absolute_position, position_info.confidence_level);
        break;
    case ENCODER_RESULT_SEARCHING:
        printf("Searching...\n");
        break;
    // 其他结果处理...
}

// 6. 清理资源
destroy_encoder(encoder);
destroy_encoder_map(map_handle);
```

### 2. 内存池模式（适用于RTOS）

```c
// 为二叉树节点预分配内存池
static tree_node_t node_pool[1000];
static uint8_t debounce_buffer[10];

// 使用内存池创建编码器图案
encoder_map_handle_t map_handle = create_encoder_map_with_pool(
    pattern_data, pattern_length, 8, node_pool, 1000);

// 其他组件也可以使用预分配的缓冲区
// 具体使用方式参考各模块的创建函数
```

### 3. 日志配置

```c
// 设置日志级别
absolute_encoder_set_log_level(LOG_LEVEL_DEBUG);

// 在absolute_encoder_logger.h中可以开关日志打印
#define ENABLE_LOGGING 1  // 启用日志
```

## API接口说明

### 主要接口

- `create_encoder_map()`: 创建编码器图案对象
- `create_encoder()`: 创建编码器对象
- `process_encoder_signal()`: 处理编码器信号（主要接口）
- `get_current_position()`: 获取当前位置信息
- `get_encoder_stats()`: 获取统计信息
- `reset_encoder()`: 重置编码器状态

### 状态和结果

- `encoder_state_t`: 编码器状态（未初始化、搜索中、跟踪中、错误）
- `encoder_result_t`: 处理结果（正常、搜索中、找到位置、跟踪更新、各种错误）

## 与C++版本的区别

1. **内存管理**: 使用malloc/free替代new/delete，提供内存池选项
2. **容器**: 使用数组和手动管理替代STL容器（vector, deque等）
3. **命名空间**: 移除C++命名空间，使用前缀区分
4. **面向对象**: 使用结构体和函数指针实现封装
5. **错误处理**: 使用返回值和NULL指针表示错误

## 注意事项

1. **内存安全**: 确保正确配对create/destroy函数调用
2. **线程安全**: 库本身不提供线程同步，多线程使用需要外部同步
3. **硬件接口**: 需要实现sys_tick_get函数提供系统时钟
4. **编译器兼容**: 使用C99标准，确保编译器支持

## 示例应用

参考原始C++版本的使用案例，所有功能都已完整移植到C版本中。

## 调试和故障排除

1. 启用日志输出查看详细运行信息
2. 检查get_encoder_stats()的统计信息
3. 验证编码器图案数据的正确性
4. 确认电机步数配置与实际硬件匹配
