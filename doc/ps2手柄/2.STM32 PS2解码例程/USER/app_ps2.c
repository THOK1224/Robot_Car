/*
 * @文件描述:
 * @作者: Q
 * @Date: 2023-02-13 14:01:12
 * @LastEditTime: 2023-02-15 17:42:43
 */
#include "app_ps2.h"

/**
 * @函数描述: PS2设备控制初始化
 * @return {*}
 */
void app_ps2_init(void)
{
    ps2_init(); /* PS2引脚初始化 */
}

/**
 * @函数描述: 循环执行工作指示灯任务运行，让LED闪烁 1s跳动一次
 * @return {*}
 */
void app_ps2_run(void)
{
    static u32 time_count = 0;

    if (millis() - time_count > 50) /* 50ms检测一次 */
    {
        time_count = millis();

        ps2_write_read();         /* 读取ps2数据 */
        if (ps2_data.isConnected) /* 判断ps2是否连接 */
        {
            if (ps2_data.mode)
            {
                if (ps2_data.left_x != ps2_data.last_left_x)
                {
                    printf("\r\n left_x:%d\t", ps2_data.left_x);
                }
                if (ps2_data.left_y != ps2_data.last_left_y)
                {
                    printf("\r\n left_y:%d\t", ps2_data.left_y);
                }
                if (ps2_data.right_x != ps2_data.last_right_x)
                {
                    printf("\r\n right_x:%d\t", ps2_data.right_x);
                }
                if (ps2_data.right_y != ps2_data.last_right_y)
                {
                    printf("\r\n right_y:%d\t", ps2_data.right_y);
                }
            }
            if (ps2_data.left_up)
            {
                printf("PS2_LEFT_UP is pressed!\r\n");
            }
            else if (ps2_data.left_right)
            {
                printf("PS2_LEFT_RIGHT is pressed!\r\n");
            }
            else if (ps2_data.left_down)
            {
                printf("PS2_LEFT_DOWN is pressed!\r\n");
            }
            else if (ps2_data.left_left)
            {
                printf("PS2_LEFT_LEFT is pressed!\r\n");
            }
            else if (ps2_data.select)
            {
                printf("PS2_SELECT is pressed!\r\n");
            }
            else if (ps2_data.start)
            {
                printf("PS2_START is pressed!\r\n");
            }
            else if (ps2_data.right_up)
            {
                printf("PS2_RIGHT_UP is pressed!\r\n");
            }
            else if (ps2_data.right_right)
            {
                printf("PS2_RIGHT_RIGHT is pressed!\r\n");
            }
            else if (ps2_data.right_down)
            {
                printf("PS2_RIGHT_DOWN is pressed!\r\n");
            }
            else if (ps2_data.right_left)
            {
                printf("PS2_RIGHT_LEFT is pressed!\r\n");
            }
            else if (ps2_data.left_2)
            {
                printf("PS2_LEFT_2 is pressed!\r\n");
            }
            else if (ps2_data.right_2)
            {
                printf("PS2_RIGHT_2 is pressed!\r\n");
            }
            else if (ps2_data.left_1)
            {
                printf("PS2_LEFT_1 is pressed!\r\n");
            }
            else if (ps2_data.right_1)
            {
                printf("PS2_RIGHT_1 is pressed!\r\n");
            }
        }
    }
}
