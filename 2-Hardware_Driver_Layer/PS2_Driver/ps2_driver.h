/**
 * @file    ps2_driver.h
 * @brief   PS2 手柄驱动 (GPIO 模拟 SPI)
 * @note    DAT=PA6(Input), CMD=PA7(Output), CS=PD12(Output), CLK=PD13(Output)
 *          PS2 协议 LSB-first, ~250kHz 时钟
 */
#ifndef _PS2_DRIVER_H
#define _PS2_DRIVER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* PS2 按键掩码 (16-bit, 低电平有效, 取反后使用) */
#define PS2_BTN_SELECT   (1 << 0)
#define PS2_BTN_L3       (1 << 1)
#define PS2_BTN_R3       (1 << 2)
#define PS2_BTN_START    (1 << 3)
#define PS2_BTN_UP       (1 << 4)
#define PS2_BTN_RIGHT    (1 << 5)
#define PS2_BTN_DOWN     (1 << 6)
#define PS2_BTN_LEFT     (1 << 7)
#define PS2_BTN_L2       (1 << 8)
#define PS2_BTN_R2       (1 << 9)
#define PS2_BTN_L1       (1 << 10)
#define PS2_BTN_R1       (1 << 11)
#define PS2_BTN_TRIANGLE (1 << 12)
#define PS2_BTN_CIRCLE   (1 << 13)
#define PS2_BTN_CROSS    (1 << 14)
#define PS2_BTN_SQUARE   (1 << 15)

/**
 * @brief PS2 手柄模式
 */
typedef enum {
    PS2_MODE_DIGITAL  = 0x41,  /**< 数字模式 */
    PS2_MODE_ANALOG   = 0x73,  /**< 模拟模式 (红灯) */
    PS2_MODE_UNKNOWN  = 0xFF,
} PS2_mode_e;

/**
 * @brief PS2 手柄数据
 */
typedef struct {
    uint16_t buttons;           /**< 按键状态 (取反后, 1=按下) */
    uint16_t buttons_edge;      /**< 按键边沿 (新按下) */

    uint8_t  lx;                /**< 左摇杆 X (0=左, 128=中, 255=右) */
    uint8_t  ly;                /**< 左摇杆 Y (0=上, 128=中, 255=下) */
    uint8_t  rx;                /**< 右摇杆 X */
    uint8_t  ry;                /**< 右摇杆 Y */

    PS2_mode_e mode;            /**< 手柄模式 */
    bool connected;             /**< 是否已连接 */
} PS2_instance_t;

/**
 * @brief 初始化 PS2 手柄
 */
void PS2_init(void);

/**
 * @brief 轮询读取 PS2 数据 (周期调用, 建议 50Hz)
 */
void PS2_update(void);

/**
 * @brief 获取 PS2 数据指针
 */
const PS2_instance_t *PS2_get_data(void);

/**
 * @brief 检查按键是否按下
 */
bool PS2_is_pressed(uint16_t btn_mask);

/**
 * @brief 检查按键是否刚按下 (边沿检测)
 */
bool PS2_is_edge(uint16_t btn_mask);

/**
 * @brief 进入模拟模式 (锁定)
 */
void PS2_enter_analog_mode(void);

#endif /* _PS2_DRIVER_H */
