/**
 * @file    ps2_driver.c
 * @brief   PS2 手柄驱动实现 (GPIO 模拟 SPI)
 * @note    LSB-first, Clock ≈ 4µs half-period
 */
#include "cmsis_os2.h"

#include "error_handler.h"
#include "bsp_usart.h"
#include "bsp_dwt.h"
#include "ps2_driver.h"
#include "robot_task.h"

/* ================= GPIO 宏定义 ================= */

#define PS2_CS_LOW()    HAL_GPIO_WritePin(PS2_CS_GPIO_Port,  PS2_CS_Pin,  GPIO_PIN_RESET)
#define PS2_CS_HIGH()   HAL_GPIO_WritePin(PS2_CS_GPIO_Port,  PS2_CS_Pin,  GPIO_PIN_SET)
#define PS2_CLK_LOW()   HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET)
#define PS2_CLK_HIGH()  HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET)
#define PS2_CMD_LOW()   HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_RESET)
#define PS2_CMD_HIGH()  HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET)
#define PS2_DAT_READ()  HAL_GPIO_ReadPin(PS2_DAT_GPIO_Port,  PS2_DAT_Pin)

#define PS2_CLK_DELAY() DWT_Delay_us(1)

/* ================= 私有变量 ================= */

static PS2_instance_t ps2_inst;
static uint8_t ps2_rx_buf[9];  /* 最多 9 字节 */

/* ================= 内部函数 ================= */

/**
 * @brief 传输一个字节 (LSB-first, 双向)
 */
static uint8_t _transfer_byte(uint8_t tx) {
    uint8_t rx = 0;

    for (uint8_t i = 0; i < 8; i++) {
        /* 设置 CMD 数据位 (LSB first) */
        if (tx & (1 << i))
            PS2_CMD_HIGH();
        else
            PS2_CMD_LOW();

        /* CLK 拉低: 发送数据 */
        PS2_CLK_LOW();
        PS2_CLK_DELAY();

        /* CLK 拉高: 采样 DAT */
        PS2_CLK_HIGH();
        PS2_CLK_DELAY();

        /* 读取 DAT (LSB first) */
        if (PS2_DAT_READ() == GPIO_PIN_SET)
            rx |= (1 << i);
    }

    return rx;
}

/**
 * @brief 发送命令序列
 * @param cmd    发送缓冲
 * @param rx     接收缓冲
 * @param len    字节数
 */
static void _send_command(const uint8_t *cmd, uint8_t *rx, uint8_t len) {
    PS2_CS_LOW();
    DWT_Delay_us(16);   /* CS 低→第一个 CLK 前的等待 */

    for (uint8_t i = 0; i < len; i++) {
        rx[i] = _transfer_byte(cmd[i]);
        DWT_Delay_us(16); /* 字节间间隔 */
    }

    PS2_CS_HIGH();
    DWT_Delay_us(16);
}

/**
 * @brief 解析接收到的数据帧
 */
static void _parse_data(void) {
    /* rx[0]=0xFF(固定), rx[1]=模式, rx[2]=0x5A(固定) */
    if (ps2_rx_buf[0] != 0xFF || ps2_rx_buf[1] == 0xFF || ps2_rx_buf[2] != 0x5A) {
        ps2_inst.connected = false;
        return;
    }

    ps2_inst.connected = true;
    ps2_inst.mode = (PS2_mode_e)ps2_rx_buf[1];

    /* 按键: rx[3] 低 8 位, rx[4] 高 8 位 (低电平有效, 取反) */
    uint16_t prev_buttons = ps2_inst.buttons;
    ps2_inst.buttons = ~((uint16_t)ps2_rx_buf[3] | ((uint16_t)ps2_rx_buf[4] << 8));

    /* 边沿检测: 本次按下但上次未按 */
    ps2_inst.buttons_edge = ps2_inst.buttons & ~prev_buttons;

    /* 摇杆 (仅模拟模式有效) */
    if (ps2_inst.mode == PS2_MODE_ANALOG) {
        ps2_inst.rx = ps2_rx_buf[5];
        ps2_inst.ry = ps2_rx_buf[6];
        ps2_inst.lx = ps2_rx_buf[7];
        ps2_inst.ly = ps2_rx_buf[8];
    } else {
        ps2_inst.rx = 128;
        ps2_inst.ry = 128;
        ps2_inst.lx = 128;
        ps2_inst.ly = 128;
    }
}

/* ================= 公开接口 ================= */

void PS2_init(void) {
    ps2_inst.buttons = 0;
    ps2_inst.buttons_edge = 0;
    ps2_inst.lx = 128;
    ps2_inst.ly = 128;
    ps2_inst.rx = 128;
    ps2_inst.ry = 128;
    ps2_inst.mode = PS2_MODE_UNKNOWN;
    ps2_inst.connected = false;

    /* 确保初始状态 */
    PS2_CS_HIGH();
    PS2_CLK_HIGH();
    PS2_CMD_HIGH();

    osDelay(10);

    /* 尝试进入模拟模式 */
    PS2_enter_analog_mode();
}

void PS2_update(void) {
    /* 轮询命令: 0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 */
    static const uint8_t poll_cmd[9] = {0x01, 0x42, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00};
    // 1. 发送轮询命令并接收数据
    _send_command(poll_cmd, ps2_rx_buf, 9);
    // 2. 解析数据帧
    _parse_data();
    // 测试
    // Uart_printf(test_uart, "conn=%d,dummy1=%d, mode=%d,dummy2=%d, btn=0x%04X RX=%3d RY=%3d LX=%3d LY=%3d\r\n",
    //     ps2_inst.connected,ps2_rx_buf[0], ps2_rx_buf[1], ps2_rx_buf[2], 
    //     (uint16_t)ps2_rx_buf[3] | ((uint16_t)ps2_rx_buf[4] << 8),
    //     ps2_rx_buf[5], ps2_rx_buf[6], ps2_rx_buf[7], ps2_rx_buf[8]);
}

const PS2_instance_t *PS2_get_instance(void) {
    return &ps2_inst;
}

void PS2_enter_analog_mode(void) {
    /* 进入配置模式 */
    static const uint8_t enter_config[9]  = {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    /* 设置模拟模式 + 锁定 */
    static const uint8_t set_analog[9]    = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
    /* 退出配置模式 */
    static const uint8_t exit_config[9]   = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

    uint8_t dummy[9];

    _send_command(enter_config, dummy, 9);
    osDelay(1);
    _send_command(set_analog, dummy, 9);
    osDelay(1);
    _send_command(exit_config, dummy, 9);
    osDelay(10);
}
