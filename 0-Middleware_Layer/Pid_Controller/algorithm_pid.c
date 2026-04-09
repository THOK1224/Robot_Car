/**
 * @file    algorithm_pid.c
 * @brief   PID 控制器算法实现
 * @author  SYSU电控组
 * @note    移植自 SYSU_Hero，适配 Robot_Car (STM32G474)
 */
#include "algorithm_pid.h"
#include "bsp_dwt.h"
#include <string.h>
#include <math.h>

#define LIMIT_MAX(input, max) do { \
    if ((input) > (max)) (input) = (max); \
    else if ((input) < -(max)) (input) = -(max); \
} while(0)

void Pid_init(Pid_instance_t *pid, Pid_init_t *config)
{
    if (pid == NULL || config == NULL) return;

    memset(pid, 0, sizeof(Pid_instance_t));

    pid->kp = config->kp;
    pid->ki = config->ki;
    pid->kd = config->kd;
    pid->max_iout = config->max_iout;
    pid->max_out = config->max_out;
    pid->deadband = config->deadband;
    pid->optimization = config->optimization;
    pid->feedfoward_coefficient = config->feedfoward_coefficient;
    pid->LPF_coefficient = config->LPF_coefficient;
    pid->integral_separation_threshold = config->integral_separation_threshold;

    DWT_GetDeltaT(&pid->dwt_counter);
    pid->dt = 0.001f;
}

void Pid_reset(Pid_instance_t *pid)
{
    if (pid == NULL) return;
    pid->Iout = 0.0f;
    pid->Pout = 0.0f;
    pid->Dout = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->last_error = 0.0f;
    pid->last_measure = 0.0f;
    pid->ITerm = 0.0f;
    DWT_GetDeltaT(&pid->dwt_counter);
}

float Pid_calculate(Pid_instance_t *pid, float measure, float target)
{
    if (pid == NULL) return 0.0f;

    /* 更新 dt */
    float dt = DWT_GetDeltaT(&pid->dwt_counter);
    if (dt > 0.1f) dt = 0.1f;
    if (dt < 0.0001f) dt = 0.0001f;
    pid->dt = dt;

    /* 更新状态 */
    pid->measure = measure;
    pid->target = target;
    pid->error = pid->target - pid->measure;

    /* 死区 */
    if (fabsf(pid->error) < pid->deadband)
    {
        pid->error = 0.0f;
        pid->Output = 0.0f;
        pid->Pout = 0.0f;
        return 0.0f;
    }

    /* P 项 */
    pid->Pout = pid->kp * pid->error;

    /* I 项 */
    if (pid->optimization & PID_TRAPEZOID_INTERGRAL) {
        pid->ITerm = pid->ki * (pid->error + pid->last_error) / 2.0f * pid->dt;
    } else {
        pid->ITerm = pid->ki * pid->error * pid->dt;
    }

    /* D 项 */
    if (pid->optimization & PID_DIFFERENTIAL_GO_FIRST) {
        pid->Dout = pid->kd * (pid->last_measure - pid->measure) / pid->dt;
    } else {
        pid->Dout = pid->kd * (pid->error - pid->last_error) / pid->dt;
    }

    /* 积分分离 */
    if ((pid->optimization & PID_INTEGRAL_SEPARATION) &&
        (pid->integral_separation_threshold > 0.0f) &&
        (fabsf(pid->error) > pid->integral_separation_threshold))
    {
        pid->ITerm = 0.0f;
    }

    /* 智能抗饱和 */
    float temp_output_prediction = pid->Pout + pid->Iout + pid->ITerm + pid->Dout;
    if (pid->optimization & PID_OUTPUT_LIMIT) {
        if (temp_output_prediction > pid->max_out && pid->ITerm > 0) {
            pid->ITerm = 0;
        } else if (temp_output_prediction < -pid->max_out && pid->ITerm < 0) {
            pid->ITerm = 0;
        }
    }

    pid->Iout += pid->ITerm;

    /* 积分限幅 */
    if (pid->optimization & PID_OUTPUT_LIMIT) {
        LIMIT_MAX(pid->Iout, pid->max_iout);
    }

    /* 总输出 */
    pid->Output = pid->Pout + pid->Iout + pid->Dout;

    /* 前馈 */
    if (pid->optimization & PID_FEEDFOWARD) {
        pid->Output += pid->feedfoward_coefficient * pid->target;
    }

    /* 输出限幅 */
    if (pid->optimization & PID_OUTPUT_LIMIT) {
        LIMIT_MAX(pid->Output, pid->max_out);
    }

    /* 输出滤波 */
    if (pid->optimization & PID_OUTPUT_FILTER) {
        float alpha = pid->dt / (pid->LPF_coefficient + pid->dt);
        pid->Output = pid->Output * alpha + pid->Last_Output * (1.0f - alpha);
    }

    /* 更新历史 */
    pid->last_measure = pid->measure;
    pid->last_error = pid->error;
    pid->Last_Output = pid->Output;

    return pid->Output;
}
