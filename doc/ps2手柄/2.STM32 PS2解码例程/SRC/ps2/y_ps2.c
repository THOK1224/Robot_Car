/****************************************************************************
 *	@笔者	：	Q
 *	@日期	：	2023年2月9日
 *	@所属	：	杭州友辉科技
 *	@功能	：	存放led相关的函数
 *	@函数列表:
 *	1.	void led_init(void) -- 初始化LED信号灯
 ****************************************************************************/

#include "./ps2/y_ps2.h"

/* 数据存储数组 */
u8 ps2_buf[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
/* 判断ps2是否连接 1为连接 0断开 */
u8 ps2_isConnected = 0;

/* ps2数据结果 */
ps2_t ps2_data;

/* PS2手柄初始化 */
void ps2_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(PS2_DAT_GPIO_CLK | PS2_CMD_GPIO_CLK | PS2_CS_GPIO_CLK | PS2_CLK_GPIO_CLK, ENABLE); // 使能端口时钟

    GPIO_InitStructure.GPIO_Pin = PS2_DAT_PIN;         // 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;      // 下拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // 50M
    GPIO_Init(PS2_DAT_GPIO_PORT, &GPIO_InitStructure); // 根据设定参数初始化GPIO

    GPIO_InitStructure.GPIO_Pin = PS2_CMD_PIN;         // 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // 50M
    GPIO_Init(PS2_CMD_GPIO_PORT, &GPIO_InitStructure); // 根据设定参数初始化GPIO

    GPIO_InitStructure.GPIO_Pin = PS2_CS_PIN;         // 端口配置
    GPIO_Init(PS2_CS_GPIO_PORT, &GPIO_InitStructure); // 根据设定参数初始化GPIO

    GPIO_InitStructure.GPIO_Pin = PS2_CLK_PIN;         // 端口配置
    GPIO_Init(PS2_CLK_GPIO_PORT, &GPIO_InitStructure); // 根据设定参数初始化GPIO

    PS2_CS(1);
    PS2_CLK(1);
    PS2_CMD(1);
}

/***********************************************
    功能介绍：读写1个字节
    函数参数：无
    返回值：无
 ***********************************************/
u8 ps2_transfer(unsigned char dat)
{
    unsigned char rd_data, wt_data, i;
    wt_data = dat;
    rd_data = 0;
    for (i = 0; i < 8; i++)
    {
        PS2_CMD((wt_data & (0x01 << i)));
        PS2_CLK(1);
        delay_us(6);
        PS2_CLK(0);
        delay_us(6);
        PS2_CLK(1);
        if (PS2_DAT())
        {
            rd_data |= 0x01 << i;
        }
    }
    return rd_data;
}

/***********************************************
    功能介绍：读取手柄数据
    函数参数：无
    返回值：无
 ***********************************************/
void ps2_write_read(void)
{
    u8 i;

    PS2_CS(0);
    ps2_buf[0] = ps2_transfer(START_CMD);
    ps2_buf[1] = ps2_transfer(ASK_DAT_CMD);
    ps2_buf[2] = ps2_transfer(ps2_buf[0]);
    ps2_buf[3] = ps2_transfer(ps2_buf[0]);
    ps2_buf[4] = ps2_transfer(ps2_buf[0]);
    ps2_buf[5] = ps2_transfer(ps2_buf[0]);
    ps2_buf[6] = ps2_transfer(ps2_buf[0]);
    ps2_buf[7] = ps2_transfer(ps2_buf[0]);
    ps2_buf[8] = ps2_transfer(ps2_buf[0]);
    PS2_CS(1);

    /* PS2数据解析 */
    if ((ps2_buf[0] == 255) && (ps2_buf[2] == 90)) /* 判断ps2是否连接上 */
    {
        ps2_data.isConnected = 1;
    }
    else
    {
        ps2_data.isConnected = 0;
    }

    if (ps2_data.isConnected) /* 连接成功才运行下面数据解析 */
    {
        if (ps2_buf[1] == PS2_MODE_GRN) /* 判断PS2模式 */
        {
            ps2_data.mode = 0;
        }
        else
        {
            ps2_data.mode = 1;
        }

        if (ps2_data.mode) /* 为红灯模式读取解析数据 */
        {
            /* 保存上一次数据 */
            ps2_data.last_left_x = ps2_data.left_x;
            ps2_data.last_left_y = ps2_data.left_y;
            ps2_data.last_right_x = ps2_data.right_x;
            ps2_data.last_right_y = ps2_data.right_y;

            /* 读取数据 */
            ps2_data.right_x = (ps2_buf[5] - 0x7f);
            ps2_data.right_y = (ps2_buf[6] - 0x80);
            ps2_data.left_x = (ps2_buf[7] - 0x7f);
            ps2_data.left_y = (ps2_buf[8] - 0x80);
        }

        /* 获取左边按钮数据 */
        ps2_data.left_up = !(ps2_buf[3] & 0x10);
        ps2_data.left_right = !(ps2_buf[3] & 0x20);
        ps2_data.left_down = !(ps2_buf[3] & 0x40);
        ps2_data.left_left = !(ps2_buf[3] & 0x80);

        /* 获取右边按钮数据 */
        ps2_data.right_up = !(ps2_buf[4] & 0x10);
        ps2_data.right_right = !(ps2_buf[4] & 0x20);
        ps2_data.right_left = !(ps2_buf[4] & 0x80);
        ps2_data.right_down = !(ps2_buf[4] & 0x40);

        /* 获取前面左右各两个按键 */
        ps2_data.left_1 = !(ps2_buf[4] & 0x04);
        ps2_data.left_2 = !(ps2_buf[4] & 0x01);
        ps2_data.right_1 = !(ps2_buf[4] & 0x08);
        ps2_data.right_2 = !(ps2_buf[4] & 0x02);

        /* 获取中间两个按钮 */
        ps2_data.select = !(ps2_buf[3] & 0x01);
        ps2_data.start = !(ps2_buf[3] & 0x08);

        /* 清空数据数组值，方便下次解析数据 */
        for (i = 0; i <= 8; i++)
        {
            ps2_buf[i] = 0;
        }
    }

    // printf("%d    %d   %d      ", ps2_buf[0], ps2_buf[2], ps2_isConnected);
    // printf("%d    %d    %d    %d    %d    %d    %d    %d    %d\r\n",ps2_buf[0],ps2_buf[1],ps2_buf[2],ps2_buf[3],ps2_buf[4],ps2_buf[5],ps2_buf[6],ps2_buf[7],ps2_buf[8]);
}
