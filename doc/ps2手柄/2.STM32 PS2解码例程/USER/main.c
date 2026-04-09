#include "main.h" /* 包含各类驱动文件 */



int main(void)
{
    rcc_init();      /* 时钟初始化 */
    SysTick_Init();  /* 初始化系统嘀答定时器，1ms定时一次 */

    app_gpio_init(); /* 初始化gpio相关引脚 */
    app_uart_init(); /*  初始化相关串口 */

    app_ps2_init();

    app_setup_start(); /* 应用程序开始 */

    while (1)
    {
        app_led_run();  /* 循环执行工作指示灯 */
        app_key_run();  /* 循环运行监测按键状态 */
        app_uart_run(); /* 串口应用循环运行 */

        app_ps2_run(); /* 运行ps2检测 */
    }
}

