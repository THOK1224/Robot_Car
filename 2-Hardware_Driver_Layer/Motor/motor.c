/**
 * @file    motor.c
 * @brief   电机管理模块实现
 */
#include "motor.h"
#include "robot_definitions.h"
#include <string.h>
#include <stdio.h>

/* ================= 私有变量 ================= */

static Motor_instance_t motor_instance[4];

/* ================= 看门狗回调 ================= */

/**
 * @brief  电机离线回调 (四个电机共用)
 * @param  owner 电机实例指针 (Motor_instance_t *)
 */
static void Motor_offline_callback(void *owner)
{
    Motor_instance_t *motor = (Motor_instance_t *)owner;
    if (motor == NULL) return;

    /* 离线后清零输出和目标，防止失控 */
    motor->output = 0;
    motor->motor_pid.pid_target = 0;

    /* 重置 PID 积分，避免上线瞬间跳变 */
    Pid_reset(&motor->motor_pid.speed_pid);
    Pid_reset(&motor->motor_pid.angle_pid);
}

/* ================= 公开接口 ================= */

void Motor_init(void) {
    float *speed_fb = Yabo_motor_get_speed_feedback();
    Pid_init_t speed_cfg = {
        .kp  = 1.0f,
        .ki  = 0.0f,
        .kd  = 0.0f,
        .max_iout  = 500.0f,
        .max_out   = 1000.0f,
        .deadband  = 0.0f,
        .optimization = PID_OUTPUT_LIMIT,
    };
    Pid_init_t angle_cfg = {
        .kp  = 1.0f,
        .ki  = 0.0f,
        .kd  = 0.0f,
        .max_iout  = 500.0f,
        .max_out   = 1000.0f,
        .deadband  = 0.0f,
        .optimization = PID_OUTPUT_LIMIT,
    };
    Watchdog_init_t wdg_cfg = {
        .reload_count    = MOTOR_WDG_RELOAD,
        .callback        = Motor_offline_callback,
        .online_callback = NULL,
    };
    
    for(uint8_t i = 0; i < 4; i++) {
        /* ---------- 1. 基本参数初始化 ---------- */
        memset(&motor_instance[i], 0, sizeof(Motor_instance_t));
        snprintf(motor_instance[i].motor_name, sizeof(motor_instance[i].motor_name), "Motor_%d", i);
        motor_instance[i].deadzone_compensation = MOTOR_DEADZONE;

        /* ---------- 2. PID 控制器初始化 ---------- */
        motor_instance[i].motor_pid.close_loop = SPEED_LOOP;
        motor_instance[i].motor_pid.pid_target       = 0;
        motor_instance[i].motor_pid.speed_feedforward = 0;
        motor_instance[i].motor_pid.angle_feedback_ptr = NULL;
        motor_instance[i].motor_pid.speed_feedback_ptr = &speed_fb[i];

        Pid_init(&motor_instance[i].motor_pid.speed_pid, &speed_cfg);
        Pid_init(&motor_instance[i].motor_pid.angle_pid, &angle_cfg);

        /* ---------- 3. 看门狗初始化 ---------- */
        snprintf(wdg_cfg.name, sizeof(wdg_cfg.name), "Motor_%d", i);
        wdg_cfg.owner_id = &motor_instance[i];
        motor_instance[i].wdg = Watchdog_register(&wdg_cfg);
    }
}

void Motor_calculate(Motor_instance_t *motor, float target)
{
    if (motor == NULL) return;

    Motor_controller_t *ctrl = &motor->motor_pid;
    ctrl->pid_target = target;

    float output = 0;

    switch (ctrl->close_loop) {
    case OPEN_LOOP:
        output = target;
        break;
    case SPEED_LOOP: {
        float speed_measure = 0;
        if (ctrl->speed_feedback_ptr != NULL)
            speed_measure = *ctrl->speed_feedback_ptr;
        output = Pid_calculate(&ctrl->speed_pid, speed_measure, target + ctrl->speed_feedforward);
        break;
    }
    case ANGLE_LOOP: {
        float angle_measure = 0;
        if (ctrl->angle_feedback_ptr != NULL)
            angle_measure = *ctrl->angle_feedback_ptr;
        output = Pid_calculate(&ctrl->angle_pid, angle_measure, target);
        break;
    }
    case ANGLE_AND_SPEED_LOOP: {
        /* 串级: 角度环输出 → 速度环目标 */
        float angle_measure = 0;
        if (ctrl->angle_feedback_ptr != NULL)
            angle_measure = *ctrl->angle_feedback_ptr;
        float speed_target = Pid_calculate(&ctrl->angle_pid, angle_measure, target);
        float speed_measure = 0;
        if (ctrl->speed_feedback_ptr != NULL)
            speed_measure = *ctrl->speed_feedback_ptr;
        output = Pid_calculate(&ctrl->speed_pid, speed_measure, speed_target + ctrl->speed_feedforward);
        break;
    }
    default:
        output = 0;
        break;
    }

    /* 死区补偿 */
    if (motor->deadzone_compensation != 0) {
        if (output > 0)
            output += motor->deadzone_compensation;
        else if (output < 0)
            output -= motor->deadzone_compensation;
    }
    motor->output = output;
}

Motor_instance_t* Motor_get_instance(void) {
    return motor_instance;
}
