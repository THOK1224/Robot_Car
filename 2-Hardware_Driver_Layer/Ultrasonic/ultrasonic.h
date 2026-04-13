/**
 * @file    ultrasonic.h
 * @brief   超声波测距模块 (EXTI 双边沿 + TIM7 计数器 + 卡尔曼滤波)
 * @note    Echo=PC7 (EXTI9_5), Trig=PC8
 *          中断只负责更新计数值，update 负责单位转换和滤波
 */
#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include "main.h"
#include "algorithm_kf.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 超声波模块实例
 */
typedef struct {
    /* 中断中更新 */
    uint32_t echo_count;        /**< 最新的 Echo 时间 (TIM7 计数值) */
    bool     echo_captured;     /**< 新数据标志 (中断中设置) */

    /* update 中更新 */
    float    raw_distance_cm;   /**< 原始距离 (单位转换后) */
    float    distance_cm;       /**< 滤波后距离 */

    /* 卡尔曼滤波器 */
    Kf_state_t kf;              /**< 1D 卡尔曼滤波器状态 */

    /* 工作变量 */
    bool measuring;             /**< 正在测量中标志 */
} Ultrasonic_state_t;

/**
 * @brief 初始化超声波模块 (启动 TIM7, 初始化 KF)
 */
void Ultrasonic_init(void);

/**
 * @brief 更新超声波数据 (在任务中周期调用)
 * @note  完成 Echo 计数 → 距离单位转换 → 卡尔曼滤波 → 数据更新
 */
void Ultrasonic_update(void);

/**
 * @brief 获取滤波后的距离 (cm)
 */
float Ultrasonic_get_distance(void);

/**
 * @brief 获取原始距离 (cm)
 */
float Ultrasonic_get_raw_distance(void);

/**
 * @brief EXTI 回调处理 (在 HAL_GPIO_EXTI_Callback 中调用)
 */
void Ultrasonic_EXTI_callback(uint16_t GPIO_Pin);

#endif /* _ULTRASONIC_H */
