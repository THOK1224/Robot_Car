/**
 * @file    ps2_control.h
 * @brief   PS2 手柄控制逻辑框架
 * @note    通过 L1 按键循环切换底盘模式，摇杆控制底盘运动
 */
#ifndef _PS2_CONTROL_H
#define _PS2_CONTROL_H

#include "chassis.h"

#pragma pack(push, 1)

/**
 * @brief 控制模式
 */
typedef enum {
    PS2_CTRL_MODE_IDLE   = 0,  /**< 空闲 */
    PS2_CTRL_MODE_MANUAL,      /**< 手动遥控 */
    PS2_CTRL_MODE_AUTO,        /**< 自动 (预留) */
} PS2_control_mode_e;

/**
 * @brief 底盘控制命令
 */
typedef struct {
    float chassis_vx;            /**< X 方向速度 (mm/s, 前为正) */
    float chassis_vy;            /**< Y 方向速度 (mm/s, 左为正) */
    float chassis_wz;            /**< 旋转角速度 (rad/s, 逆时针为正) */
    Chassis_mode_e chassis_mode; /**< 底盘模式 */
} Chassis_cmd_t;

/**
 * @brief PS2 控制实例
 */
typedef struct {
    PS2_control_mode_e mode;   /**< 当前的控制模式 */
    Chassis_cmd_t cmd;         /**< 输出给执行层的指令 */
    float speed_scale;         /**< 速度缩放系数 */
} PS2_control_instance_t;

#pragma pack(pop)

/**
 * @brief 初始化 PS2 控制
 */
void PS2_Control_init(void);

/**
 * @brief 更新控制逻辑 (周期调用)
 */
void PS2_Control_update(void);

#endif /* _PS2_CONTROL_H */
