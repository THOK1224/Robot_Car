/**
 * @file    ps2_control.h
 * @brief   PS2 手柄控制逻辑框架
 * @note    仅搭建框架，具体按键→动作映射由用户后续填充
 */
#ifndef _PS2_CONTROL_H
#define _PS2_CONTROL_H

#include "chassis.h"
#include "ps2_driver.h"

/**
 * @brief 控制模式
 */
typedef enum {
    PS2_CTRL_MODE_IDLE   = 0,  /**< 空闲 */
    PS2_CTRL_MODE_MANUAL,      /**< 手动遥控 */
    PS2_CTRL_MODE_AUTO,        /**< 自动 (预留) */
} PS2_control_mode_e;

/**
 * @brief PS2 控制实例
 */
typedef struct {
    PS2_control_mode_e mode;   /**< 当前模式 */
    Chassis_cmd_t      cmd;    /**< 输出给底盘的指令 */
    float speed_scale;         /**< 速度缩放系数 */
} PS2_control_instance_t;

/**
 * @brief 初始化 PS2 控制
 */
void PS2_Control_init(void);

/**
 * @brief 更新控制逻辑 (周期调用)
 */
void PS2_Control_update(void);

/**
 * @brief 获取底盘指令
 */
const Chassis_cmd_t *PS2_Control_get_cmd(void);

/**
 * @brief 获取当前模式
 */
PS2_control_mode_e PS2_Control_get_mode(void);

#endif /* _PS2_CONTROL_H */
