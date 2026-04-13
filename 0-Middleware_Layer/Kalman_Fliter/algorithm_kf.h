/**
 * @file    algorithm_kf.h
 * @brief   1D 卡尔曼滤波器 (用于距离/温度等标量值)
 * @note    与公共层 EKF 区别：KF 用于 1D 标量，EKF 用于四元数/姿态估计
 */
#ifndef _ALGORITHM_KF_H
#define _ALGORITHM_KF_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 1D 卡尔曼滤波器状态
 */
typedef struct {
    float x;                    /*!< 状态估计 (滤波值) */
    float p;                    /*!< 估计协方差 */
    float q;                    /*!< 过程噪声 */
    float r;                    /*!< 测量噪声 */
    float k;                    /*!< 卡尔曼增益 (调试用) */
    bool is_initialized;        /*!< 初始化标志 */
} Kf_state_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 1D 卡尔曼滤波器
 * @param kf 滤波器状态结构体
 * @param q  过程噪声 (越大跟踪越快)
 * @param r  测量噪声 (越大越平滑)
 */
void Kf_init(Kf_state_t *kf, float q, float r);

/**
 * @brief 卡尔曼滤波更新
 * @param kf          滤波器状态结构体
 * @param measurement 新测量值
 * @return 滤波后的状态估计值
 */
float Kf_update(Kf_state_t *kf, float measurement);

/**
 * @brief 重置滤波器状态
 * @param kf    滤波器状态结构体
 * @param value 重置到的初始值
 */
void Kf_reset(Kf_state_t *kf, float value);

#ifdef __cplusplus
}
#endif

#endif /* _ALGORITHM_KF_H */
