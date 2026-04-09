/**
 * @file    error_port.c
 * @brief   错误处理系统平台移植层
 * @author  SYSU电控组
 * @note    移植自 SYSU_Hero，已移除 RTT 和蜂鸣器，仅 UART 输出
 */
#include "error_handler.h"
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "main.h"
#include "bsp_usart.h"

extern Uart_instance_t* test_uart;

/* 内部函数声明 */
void* error_get_uart_handle(void);
void error_set_critical_flag(void);

/* ===================== 工具函数 ===================== */

static char error_output_buf[256];

static uint8_t error_port_in_isr(void)
{
    return (__get_IPSR() != 0u) ? 1u : 0u;
}

/* ===================== 平台接口实现 ===================== */

uint16_t error_port_get_task_id(void)
{
    if (error_port_in_isr()) return 0u;

    if (osKernelGetState() == osKernelRunning)
    {
        osThreadId_t tid = osThreadGetId();
        if (tid != NULL)
            return (uint16_t)((uintptr_t)tid & 0xFFFFu);
    }
    return 0u;
}

uint32_t error_port_get_timestamp(void)
{
    return HAL_GetTick();
}

void error_port_output(const error_record_t* record)
{
    if (record == NULL) return;

    static const char* level_str[] = { "INFO", "WARN", "ERR", "CRIT" };

    uint8_t level = ERROR_GET_LEVEL(record->error_code);
    uint8_t in_isr = error_port_in_isr();
    if (level > ERROR_LEVEL_CRITICAL) level = ERROR_LEVEL_ERROR;

    memset(error_output_buf, 0, sizeof(error_output_buf));

    if (record->function != NULL)
    {
        snprintf(error_output_buf, sizeof(error_output_buf),
                 "[%s][%s] %s:%lu %s",
                 level_str[level],
                 record->module_name ? record->module_name : "UNK",
                 record->function,
                 record->line,
                 record->message);
    }
    else
    {
        snprintf(error_output_buf, sizeof(error_output_buf),
                 "[%s][%s] %s",
                 level_str[level],
                 record->module_name ? record->module_name : "UNK",
                 record->message);
    }

    /* UART 输出 (非中断上下文) */
    strcat(error_output_buf, "\r\n");
    Uart_instance_t* uart = (Uart_instance_t*)error_get_uart_handle();
    if (uart != NULL && !in_isr)
    {
        Uart_printf(uart, "%s", error_output_buf);
    }

    /* Critical 处理 */
    if (level == ERROR_LEVEL_CRITICAL)
    {
        error_set_critical_flag();
    }
}
