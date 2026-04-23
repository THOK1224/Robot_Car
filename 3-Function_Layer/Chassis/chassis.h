/**
 * @file    chassis.h
 * @brief   麦克纳姆轮底盘运动学模块
 * @note    仅包含麦轮逆运动学解算
 *          底盘参数: wheel_radius=97mm, wheel_base=250mm, track_width=250mm
 */
#ifndef _CHASSIS_H
#define _CHASSIS_H

#include <stdint.h>

#pragma pack(push, 1)

/**
 * @brief 底盘模式
 */
typedef enum {
    CHASSIS_ZERO_FORCE = 0,  /**< 零力模式 (电机不输出) **/
    CHASSIS_TRANSLATION,     /**< 底盘平动运动 **/
    CHASSIS_ROTATE,          /**< 底盘旋转运动 **/
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
 * @brief 底盘输出 (4 轮目标速度)
 */
typedef struct {
    float motor_speed[4];   /**< M1~M4 目标速度 */
} Chassis_output_t;

#pragma pack(pop)

/**
 * @brief 初始化底盘模块 (参数、电机、消息中心)
 */
void Chassis_init(void);

/**
 * @brief 底盘周期更新 (从队列获取指令 → 解算 → PID → 发布)
 */
void Chassis_update(void);

#endif /* _CHASSIS_H */
