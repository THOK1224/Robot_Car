/**
 * @file    ultrasonic.h
 * @brief   超声波测距模块 (EXTI 双边沿 + TIM7 计数器)
 * @note    Echo=PC7 (EXTI9_5), Trig=PC8, TIM7 1µs/tick
 */
#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 超声波测量状态
 */
typedef enum {
    US_STATE_IDLE = 0,       /**< 空闲 */
    US_STATE_WAIT_ECHO,      /**< 等待回声上升沿 */
    US_STATE_MEASURING,      /**< 测量中 (等待下降沿) */
    US_STATE_DONE,           /**< 测量完成 */
    US_STATE_TIMEOUT,        /**< 超时 */
} Ultrasonic_state_e;

/**
 * @brief 1D 卡尔曼滤波器状态
 */
typedef struct {
    float x;    /**< 状态估计 */
    float p;    /**< 估计协方差 */
    float q;    /**< 过程噪声 */
    float r;    /**< 测量噪声 */
    float k;    /**< 卡尔曼增益 */
} Kalman1D_t;

/**
 * @brief 超声波模块实例
 */
typedef struct {
    Ultrasonic_state_e state;
    uint32_t echo_count;       /**< TIM7 计数值 */
    float    raw_distance_cm;  /**< 原始距离 (cm) */
    float    distance_cm;      /**< 滤波后距离 (cm) */
    bool     data_ready;       /**< 新数据就绪标志 */
    uint32_t timeout_count;    /**< 超时计数 */

    Kalman1D_t kalman;         /**< 卡尔曼滤波器 */
} Ultrasonic_instance_t;

/**
 * @brief 初始化超声波模块 (启动 TIM7)
 */
void Ultrasonic_init(void);

/**
 * @brief 触发一次测量 (发送 Trig 脉冲)
 */
void Ultrasonic_trigger(void);

/**
 * @brief 更新距离 (在任务中周期调用)
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
