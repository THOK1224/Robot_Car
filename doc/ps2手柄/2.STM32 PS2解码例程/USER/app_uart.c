/*
 * @文件描述:
 * @作者: Q
 * @Date: 2023-02-13 14:01:12
 * @LastEditTime: 2023-02-15 15:39:59
 */
#include "app_uart.h"

/**
 * @函数描述: uart串口相关设备控制初始化
 * @return {*}
 */
void app_uart_init(void)
{
    uart1_init(115200); /* uart1串口初始化 */
}

/**
 * @函数描述: 循环检测串口接收到的指令
 * @return {*}
 */
void app_uart_run(void)
{
    if (uart1_get_ok & 0x8000)
    {
        uart1_get_ok = 0;
        printf("received ok success! = %s\r\n",uart_receive_buf);
    }
}
