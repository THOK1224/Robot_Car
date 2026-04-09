/**
 * @file    chassis.h
 * @brief   麦克纳姆轮底盘运动学模块
 * @note    仅包含麦轮逆运动学解算
 *          底盘参数: wheel_radius=97mm, wheel_base=250mm, track_width=250mm
 */
#ifndef _CHASSIS_H
#define _CHASSIS_H

#include <stdint.h>

/**
 * @brief 底盘模式
 */
typedef enum {
    CHASSIS_ZERO_FORCE = 0,  /**< 零力模式 (电机不输出) */
    CHASSIS_NO_FOLLOW,       /**< 正常底盘运动 */
} Chassis_mode_e;

/**
 * @brief 底盘物理参数
 */
typedef struct {
    float wheel_radius_mm;   /**< 轮半径 (mm) */
    float wheel_base_mm;     /**< 前后轮距 (mm) */
    float track_width_mm;    /**< 左右轮距 (mm) */
} Chassis_params_t;

/**
 * @brief 底盘速度指令
 */
typedef struct {
    float vx;       /**< X 方向速度 (mm/s, 前为正) */
    float vy;       /**< Y 方向速度 (mm/s, 左为正) */
    float omega;    /**< 旋转角速度 (rad/s, 逆时针为正) */
} Chassis_cmd_t;

/**
 * @brief 底盘输出 (4 轮目标速度)
 */
typedef struct {
    int16_t speed[4];   /**< M1~M4 目标速度 (-1000~1000) */
} Chassis_output_t;

/**
 * @brief 初始化底盘模块
 * @param params 底盘物理参数
 */
void Chassis_init(const Chassis_params_t *params);

/**
 * @brief 运动学解算 (vx,vy,omega → 4 轮速)
 * @param cmd    速度指令输入
 * @param output 4 轮速度输出
 */
void Chassis_update(const Chassis_cmd_t *cmd, Chassis_output_t *output);

/**
 * @brief 设置底盘模式
 */
void Chassis_set_mode(Chassis_mode_e mode);

/**
 * @brief 获取底盘模式
 */
Chassis_mode_e Chassis_get_mode(void);

#endif /* _CHASSIS_H */
