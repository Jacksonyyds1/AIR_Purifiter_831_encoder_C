#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include <time.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// 日志级别定义
typedef enum
{
    LOG_LEVEL_VERBOSE = 0,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR
} LogLevel;

// 全局日志级别，可以在运行时调整
extern LogLevel g_log_level;

// 获取当前时间字符串
static inline void get_timestamp(char *buffer, size_t size)
{
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    strftime(buffer, size, "%Y-%m-%d %H:%M:%S", tm_info);
}

// 获取文件名（去除路径）
static inline const char *get_filename(const char *filepath)
{
    const char *filename = strrchr(filepath, '/');
    return filename ? filename + 1 : filepath;
}

// 通用日志打印函数
#define LOG_PRINT(level, tag, fmt, ...) do { \
        if (level >= g_log_level) { \
            char timestamp[32]; \
            get_timestamp(timestamp, sizeof(timestamp)); \
            printf("[%s] %s %s:%d %s() " fmt "\n", \
                   timestamp, tag, get_filename(__FILE__), __LINE__, __func__, ##__VA_ARGS__); \
            fflush(stdout); \
        } \
    } while(0)

// 分级日志宏定义
#define LOGV(fmt, ...) LOG_PRINT(LOG_LEVEL_VERBOSE, "[V]", fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) LOG_PRINT(LOG_LEVEL_DEBUG,   "[D]", fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) LOG_PRINT(LOG_LEVEL_INFO,    "[I]", fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) LOG_PRINT(LOG_LEVEL_WARN,    "[W]", fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) LOG_PRINT(LOG_LEVEL_ERROR,   "[E]", fmt, ##__VA_ARGS__)

// 条件日志宏
#define LOGV_IF(cond, fmt, ...) do { if (cond) LOGV(fmt, ##__VA_ARGS__); } while(0)
#define LOGD_IF(cond, fmt, ...) do { if (cond) LOGD(fmt, ##__VA_ARGS__); } while(0)
#define LOGI_IF(cond, fmt, ...) do { if (cond) LOGI(fmt, ##__VA_ARGS__); } while(0)
#define LOGW_IF(cond, fmt, ...) do { if (cond) LOGW(fmt, ##__VA_ARGS__); } while(0)
#define LOGE_IF(cond, fmt, ...) do { if (cond) LOGE(fmt, ##__VA_ARGS__); } while(0)

// 设置日志级别的函数
static inline void set_log_level(LogLevel level)
{
    g_log_level = level;
}

// 初始化日志系统（默认级别为INFO）
static inline void log_init()
{
    g_log_level = LOG_LEVEL_INFO;
}

#ifdef __cplusplus
}
#endif

#endif // LOGGER_H