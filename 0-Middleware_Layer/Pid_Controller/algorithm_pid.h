/**
 * @file    algorithm_pid.h
 * @brief   PID 控制器算法模块
 * @author  SYSU电控组
 * @note    移植自 SYSU_Hero，适配 Robot_Car (STM32G474)
 */
#ifndef _ALGORITHM_PID_H
#define _ALGORITHM_PID_H

#include "main.h"
#include <stdint.h>

/**
 * @brief PID 优化环节标志 (位掩码)
 */
typedef enum
{
    PID_OPTIMIZE_NONE           = 0x00, // 无优化
    PID_OUTPUT_LIMIT            = 0x01, // 输出限幅 (优先级最高)
    PID_DIFFERENTIAL_GO_FIRST   = 0x02, // 微分先行 (适用于测量噪声较大时)
    PID_TRAPEZOID_INTERGRAL     = 0x04, // 梯形积分 (误差变化较大时防止积分过冲)
    PID_OUTPUT_FILTER           = 0x08, // 输出低通滤波 (适用于执行机构响应慢时)
    PID_FEEDFOWARD              = 0x10, // 前馈控制 (适用于存在明显可测扰动时，如底盘反向速度补偿)
    PID_INTEGRAL_ANTI_WINDUP    = 0x20, // 积分反向保护 (适用于负载变化大时防止积分反向过度补偿)
    PID_INTEGRAL_SEPARATION     = 0x40, // 积分分离 (适用于大误差时防止积分过度积累)
} Pid_optimization_e;

/**
 * @brief PID 运行时实例
 */
typedef struct
{
    float kp;
    float ki;
    float kd;

    float max_iout;
    float max_out;
    float deadband;

    float measure;
    float last_measure;
    float target;
    float error;
    float last_error;

    float ITerm;
    float Pout;
    float Iout;
    float Dout;
    float Output;

    float Last_Output;

    uint32_t optimization;
    float feedfoward_coefficient;
    float LPF_coefficient;
    float integral_separation_threshold;

    uint32_t dwt_counter;
    float    dt;
} Pid_instance_t;

/**
 * @brief PID 初始化配置
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float max_iout;
    float max_out;
    float deadband;

    uint32_t optimization;
    float feedfoward_coefficient;
    float LPF_coefficient;
    float integral_separation_threshold;
} Pid_init_t;

/**
 * @brief 初始化 PID 实例
 */
void Pid_init(Pid_instance_t *pid, Pid_init_t *config);

/**
 * @brief 计算 PID 输出
 * @return PID 计算输出值
 */
float Pid_calculate(Pid_instance_t *pid, float measure, float target);

/**
 * @brief 重置 PID 状态 (积分清零)
 */
void Pid_reset(Pid_instance_t *pid);

#endif //_ALGORITHM_PID_H
