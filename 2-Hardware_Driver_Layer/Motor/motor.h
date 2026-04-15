/**
 * @file    motor.h
 * @brief   电机管理模块 (最多 6 个电机实例)
 * @note    结构体定义位于 chassis.h (Motor_instance_t / Motor_controller_t)
 *          本模块负责电机注册、PID 闭环计算、看门狗离线保护
 */
#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>
#include "algorithm_pid.h"
#include "bsp_wdg.h"

/* 看门狗超时重装值 (单位: Watchdog_control_all 调用次数) */
#define MOTOR_WDG_RELOAD 200

#pragma pack(push, 1)

/**电机控制类型**/
typedef enum
{
    OPEN_LOOP = 0b0000,             //开环
    SPEED_LOOP = 0b0001,            //速度环
    ANGLE_LOOP = 0b0010,            //角度环
    ANGLE_AND_SPEED_LOOP = 0b0011,  //角度+速度环
} Motor_closeloop_e;

/**电机PID控制器结构体**/
typedef struct
{
    Motor_closeloop_e close_loop;       //当前电机模式
    float *angle_feedback_ptr; // 角度反馈数据指针
    float *speed_feedback_ptr; // 速度反馈数据指针
    Pid_instance_t angle_pid;   //角度环
    Pid_instance_t speed_pid;   //速度环
    float pid_target;           //PID目标量

    // 速度前馈量，用于存放底盘反向速度等外部干扰补偿
    float speed_feedforward;
} Motor_controller_t;

/**电机实例结构体**/
typedef struct
{
    char motor_name[16];  // 电机名称 
    Motor_controller_t  motor_pid;  //电机自身的PID控制器
    Watchdog_device_t *wdg;        //电机看门狗实例指针
    int16_t deadzone_compensation;    // 电机死区补偿值
    float output;  // 缓存计算出的输出 (等待发送)
} Motor_instance_t;

#pragma pack(pop)

/**
 * @brief  注册一个电机实例 (最多 MOTOR_MAX_NUM 个)
 * @note   内部会初始化 PID、看门狗等子模块
 */
void Motor_init(void);

/**
 * @brief  根据目标值计算电机输出
 * @param  motor  电机实例指针
 * @param  target 目标值 (含义取决于 close_loop 模式)
 * @note   计算结果写入 motor->output
 */
void Motor_calculate(Motor_instance_t* motor, float target);

Motor_instance_t* Motor_get_instance(void);

#endif /* _MOTOR_H */
