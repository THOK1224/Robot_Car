/**
 * @file    error_handler.c
 * @brief   错误处理系统核心实现
 * @author  SYSU电控组
 * @note    移植自 SYSU_Hero，已移除 SEGGER_RTT 依赖
 */
#include "error_handler.h"
#include <string.h>
#include <stdio.h>
#include "cmsis_gcc.h"

#define ERROR_BUFFER_MASK   (ERROR_BUFFER_SIZE - 1u)
#define ERROR_BUFFER_INDEX(i) ((i) & ERROR_BUFFER_MASK)

/* ===================== 私有变量 ===================== */

static error_record_t error_buffer[ERROR_BUFFER_SIZE];
static volatile uint32_t error_head = 0u;
static volatile uint32_t error_count = 0u;
static error_system_status_t error_status;
static volatile bool error_has_critical_flag = false;
static void* error_uart_handle = NULL;

/* ===================== 工具函数 ===================== */

static void error_buffer_push(const error_record_t* record)
{
    __disable_irq();

    error_buffer[ERROR_BUFFER_INDEX(error_head)] = *record;
    error_head++;

    if (error_count < ERROR_BUFFER_SIZE)
        error_count++;
    else
        error_status.overflow_count++;

    error_status.total_count++;

    __enable_irq();
}

/* ===================== 接口实现 ===================== */

void error_system_init(void* uart_handle)
{
    memset(error_buffer, 0, sizeof(error_buffer));
    error_head = 0u;
    error_count = 0u;
    memset(&error_status, 0, sizeof(error_status));
    error_has_critical_flag = false;
    error_uart_handle = uart_handle;
}

void error_report_core(error_level_t level,
                       const char* module_name,
                       const char* function,
                       uint32_t line,
                       const char* format, ...)
{
    static char formatted_message[128];
    va_list args;
    error_record_t record;

    va_start(args, format);
    vsnprintf(formatted_message, sizeof(formatted_message), format, args);
    va_end(args);

    memset(&record, 0, sizeof(record));
    record.error_code = MAKE_ERROR_CODE(level);
    record.module_name = module_name;
    record.timestamp = error_port_get_timestamp();
    record.function = function;
    record.line = line;
    record.message = formatted_message;
    record.task_id = error_port_get_task_id();

    error_buffer_push(&record);
    error_port_output(&record);
}

uint32_t error_get_total_count(void)
{
    return error_status.total_count;
}

const error_record_t* error_get_latest(void)
{
    if (error_count == 0u) return NULL;
    return &error_buffer[ERROR_BUFFER_INDEX(error_head - 1u)];
}

const error_record_t* error_get_history(uint32_t index)
{
    if (error_count == 0u || index >= error_count) return NULL;
    return &error_buffer[ERROR_BUFFER_INDEX(error_head - 1u - index)];
}

void error_get_system_status(error_system_status_t* status)
{
    if (status == NULL) return;
    __disable_irq();
    *status = error_status;
    __enable_irq();
}

void error_clear_records(void)
{
    __disable_irq();
    memset(error_buffer, 0, sizeof(error_buffer));
    error_head = 0u;
    error_count = 0u;
    __enable_irq();
}

bool error_has_critical(void)
{
    return error_has_critical_flag;
}

void* error_get_uart_handle(void)
{
    return error_uart_handle;
}

void error_set_critical_flag(void)
{
    error_has_critical_flag = true;
}
