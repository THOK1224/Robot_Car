/****************************************************************************
 *	@笔者	：	Q
 *	@日期	：	2023年2月9日
 *	@所属	：	杭州友辉科技
 *	@功能	：	存放led相关的函数
 ****************************************************************************/

#ifndef _Y_PS2_H_
#define _Y_PS2_H_
#include "main.h"

extern u8 ps2_buf[9];
extern u8 ps2_isConnected;

/* 定义PS2引脚，修改编号就可以修改PS2引脚 */
#define PS2_DAT_PIN GPIO_Pin_4
#define PS2_DAT_GPIO_PORT GPIOB               /* GPIO端口 */
#define PS2_DAT_GPIO_CLK RCC_APB2Periph_GPIOC /* GPIO端口时钟 */

#define PS2_CMD_PIN GPIO_Pin_5
#define PS2_CMD_GPIO_PORT GPIOB               /* GPIO端口 */
#define PS2_CMD_GPIO_CLK RCC_APB2Periph_GPIOB /* GPIO端口时钟 */

#define PS2_CS_PIN GPIO_Pin_6
#define PS2_CS_GPIO_PORT GPIOB               /* GPIO端口 */
#define PS2_CS_GPIO_CLK RCC_APB2Periph_GPIOC /* GPIO端口时钟 */

#define PS2_CLK_PIN GPIO_Pin_7
#define PS2_CLK_GPIO_PORT GPIOB               /* GPIO端口 */
#define PS2_CLK_GPIO_CLK RCC_APB2Periph_GPIOB /* GPIO端口时钟 */

/*******PS2相关指令表*******/
#define START_CMD 0x01   /* 开始命令 */
#define ASK_DAT_CMD 0x42 /* 请求数据 */

/*******PS2模式数据表*******/
#define PS2_MODE_GRN 0x41 /* 模拟绿灯 */
#define PS2_MODE_RED 0x73 /* 模拟红灯 */

/* 控制PS2的宏 */
#define PS2_DAT() GPIO_ReadInputDataBit(PS2_DAT_GPIO_PORT, PS2_DAT_PIN)        // 读取LED信号灯状态
#define PS2_CMD(x) GPIO_WriteBit(PS2_CMD_GPIO_PORT, PS2_CMD_PIN, (BitAction)x) // 翻转LED信号灯
#define PS2_CS(x) GPIO_WriteBit(PS2_CS_GPIO_PORT, PS2_CS_PIN, (BitAction)x)    // 翻转LED信号灯
#define PS2_CLK(x) GPIO_WriteBit(PS2_CLK_GPIO_PORT, PS2_CLK_PIN, (BitAction)x) // 翻转LED信号灯

/*******PS2按键检测表*******/
#define PS2_LEFT_UP !(ps2_buf[3] & 0x10)
#define PS2_LEFT_RIGHT !(ps2_buf[3] & 0x20)
#define PS2_LEFT_DOWN !(ps2_buf[3] & 0x40)
#define PS2_LEFT_LEFT !(ps2_buf[3] & 0x80)

#define PS2_SELECT !(ps2_buf[3] & 0x01)
#define PS2_START !(ps2_buf[3] & 0x08)

#define PS2_RIGHT_UP !(ps2_buf[4] & 0x10)
#define PS2_RIGHT_RIGHT !(ps2_buf[4] & 0x20)
#define PS2_RIGHT_DOWN !(ps2_buf[4] & 0x40)
#define PS2_RIGHT_LEFT !(ps2_buf[4] & 0x80)

#define PS2_LEFT_2 !(ps2_buf[4] & 0x01)
#define PS2_RIGHT_2 !(ps2_buf[4] & 0x02)
#define PS2_LEFT_1 !(ps2_buf[4] & 0x04)
#define PS2_RIGHT_1 !(ps2_buf[4] & 0x08)

#define PS2_RIGHT_X (int)(ps2_buf[5] - 0x7f)
#define PS2_RIGHT_Y (int)(ps2_buf[6] - 0x80)
#define PS2_LEFT_X (int)(ps2_buf[7] - 0x7f)
#define PS2_LEFT_Y (int)(ps2_buf[8] - 0x80)

#define PS2_MODE ps2_buf[1]

typedef struct
{
    /* 左边4个按键 1按下 */
    u8 left_up;
    u8 left_right;
    u8 left_down;
    u8 left_left;

    /* 右边4个按键 1按下 */
    u8 right_up;
    u8 right_right;
    u8 right_down;
    u8 right_left;

    /* 中间两个按键 1按下 */
    u8 select;
    u8 start;

    /* 前面左右各两个按键 1按下 */
    u8 left_1;
    u8 left_2;
    u8 right_1;
    u8 right_2;

    /* 红绿灯模式，0绿灯 1红灯 */
    u8 mode;

    /* 红灯模式下的数据 */
    short right_x;
    short right_y;
    short left_x;
    short left_y;
    short last_right_x; /* 上一次获取的数值 */
    short last_right_y;
    short last_left_x;
    short last_left_y;

    /* 判断是否连接上 1连接 0断开 */
    u8 isConnected;
} ps2_t;

extern ps2_t ps2_data;

/*******PS2相关函数声明*******/
void ps2_init(void);       /* PS2手柄初始化 */
void ps2_write_read(void); /* 读取手柄数据 */
#endif
