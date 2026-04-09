/**
 * @file    error_handler.h
 * @brief   错误处理系统核心接口
 * @note    移植自 SYSU_Hero，已移除 RTT 和蜂鸣器依赖
 */
#ifndef __ERROR_HANDLER_H
#define __ERROR_HANDLER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdarg.h>

#define ERROR_BUFFER_SIZE       32u

/**
 * @brief 错误等级枚举
 */
typedef enum {
    ERROR_LEVEL_INFO     = 0u,
    ERROR_LEVEL_WARNING  = 1u,
    ERROR_LEVEL_ERROR    = 2u,
    ERROR_LEVEL_CRITICAL = 3u,
} error_level_t;

/**
 * @brief 错误记录结构体
 */
typedef struct {
    uint32_t    error_code;
    const char *module_name;
    uint32_t    timestamp;
    const char *function;
    uint32_t    line;
    const char *message;
    uint16_t    task_id;
    uint8_t     cpu_id;
    uint8_t     reserved;
} error_record_t;

/**
 * @brief 系统状态结构体
 */
typedef struct {
    uint32_t total_count;
    uint32_t overflow_count;
} error_system_status_t;

#define MAKE_ERROR_CODE(level)  ((uint32_t)((level) & 0xFFu))
#define ERROR_GET_LEVEL(code)   ((code) & 0xFFu)

void error_system_init(void *uart_handle);

void error_report_core(error_level_t level,
                       const char *module_name,
                       const char *function,
                       uint32_t line,
                       const char *format, ...);

#define ERROR_INFO(module_name, fmt, ...) \
    error_report_core(ERROR_LEVEL_INFO, module_name, __func__, __LINE__, fmt, ##__VA_ARGS__)

#define ERROR_WARN(module_name, fmt, ...) \
    error_report_core(ERROR_LEVEL_WARNING, module_name, __func__, __LINE__, fmt, ##__VA_ARGS__)

#define ERROR_RAISE(module_name, fmt, ...) \
    error_report_core(ERROR_LEVEL_ERROR, module_name, __func__, __LINE__, fmt, ##__VA_ARGS__)

#define ERROR_CRITICAL(module_name, fmt, ...) \
    error_report_core(ERROR_LEVEL_CRITICAL, module_name, __func__, __LINE__, fmt, ##__VA_ARGS__)

uint32_t error_get_total_count(void);
const error_record_t *error_get_latest(void);
const error_record_t *error_get_history(uint32_t index);
void error_get_system_status(error_system_status_t *status);
void error_clear_records(void);
bool error_has_critical(void);

/* 平台相关接口 */
uint16_t error_port_get_task_id(void);
uint32_t error_port_get_timestamp(void);
void error_port_output(const error_record_t *record);

#endif /* __ERROR_HANDLER_H */
