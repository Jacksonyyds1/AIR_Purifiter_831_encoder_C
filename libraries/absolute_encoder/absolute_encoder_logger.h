#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

// 日志级别定义
typedef enum
{
    LOG_LEVEL_ERROR = 0,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_VERBOSE
} LogLevel;

#define ENABLE_LOGGING 0 // 是否启用日志打印

#if ENABLE_LOGGING

#include <string.h>

// 获取文件名（去除路径）
static inline const char *get_filename(const char *filepath)
{
    const char *filename = strrchr(filepath, '/');
    return filename ? filename + 1 : filepath;
}

// 通用日志打印函数
#define LOG_PRINT(setting, level, tag, fmt, ...) do { \
        if (level <= setting) { \
            printf("[%s] %s:%d %s() " fmt "\n", \
                   tag, get_filename(__FILE__), __LINE__, __func__, ##__VA_ARGS__); \
            fflush(stdout); \
        } \
    } while(0)

// 分级日志宏定义
#define LOGV(level, fmt, ...) LOG_PRINT(level, LOG_LEVEL_VERBOSE, "[V]", fmt, ##__VA_ARGS__);
#define LOGD(level, fmt, ...) LOG_PRINT(level, LOG_LEVEL_DEBUG,   "[D]", fmt, ##__VA_ARGS__);
#define LOGI(level, fmt, ...) LOG_PRINT(level, LOG_LEVEL_INFO,    "[I]", fmt, ##__VA_ARGS__);
#define LOGW(level, fmt, ...) LOG_PRINT(level, LOG_LEVEL_WARN,    "[W]", fmt, ##__VA_ARGS__);
#define LOGE(level, fmt, ...) LOG_PRINT(level, LOG_LEVEL_ERROR,   "[E]", fmt, ##__VA_ARGS__);

// 条件日志宏
#define LOGV_IF(cond, level, fmt, ...) do { if (cond) LOGV(level, fmt, ##__VA_ARGS__); } while(0)
#define LOGD_IF(cond, level, fmt, ...) do { if (cond) LOGD(level, fmt, ##__VA_ARGS__); } while(0)
#define LOGI_IF(cond, level, fmt, ...) do { if (cond) LOGI(level, fmt, ##__VA_ARGS__); } while(0)
#define LOGW_IF(cond, level, fmt, ...) do { if (cond) LOGW(level, fmt, ##__VA_ARGS__); } while(0)
#define LOGE_IF(cond, level, fmt, ...) do { if (cond) LOGE(level, fmt, ##__VA_ARGS__); } while(0)

#else // 如果不启用日志打印
#define LOGV(level, fmt, ...)
#define LOGD(level, fmt, ...)
#define LOGI(level, fmt, ...)
#define LOGW(level, fmt, ...)
#define LOGE(level, fmt, ...)
#define LOGV_IF(cond, level, fmt, ...)
#define LOGD_IF(cond, level, fmt, ...)
#define LOGI_IF(cond, level, fmt, ...)
#define LOGW_IF(cond, level, fmt, ...)
#define LOGE_IF(cond, level, fmt, ...)
#endif // ENABLE_LOGGING

#ifdef __cplusplus
}
#endif

#endif // LOGGER_H
