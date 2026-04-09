/**
 * @file    chassis.c
 * @brief   麦克纳姆轮底盘运动学实现
 * @note    麦轮逆运动学:
 *          V_FL = vx - vy - (L+W)*omega
 *          V_FR = vx + vy + (L+W)*omega
 *          V_RL = vx + vy - (L+W)*omega
 *          V_RR = vx - vy + (L+W)*omega
 *          (L = wheel_base/2, W = track_width/2)
 *
 *          电机映射: M1=左前(FL), M2=左后(RL), M3=右前(FR), M4=右后(RR)
 */
#include "chassis.h"
#include "math_lib.h"
#include <string.h>

/* ================= 私有变量 ================= */

static Chassis_params_t chassis_params;
static Chassis_mode_e   chassis_mode = CHASSIS_ZERO_FORCE;
static float half_sum_LW = 0.0f;  /* (L + W) / 2, 但这里直接用 L + W 的一半 */

#define SPEED_MAX  1000
#define SPEED_MIN  (-1000)

/* ================= 公开接口 ================= */

void Chassis_init(const Chassis_params_t *params)
{
    if (params == NULL) return;

    chassis_params = *params;

    /* (wheel_base + track_width) / 2, 用于运动学系数 */
    half_sum_LW = (params->wheel_base_mm + params->track_width_mm) / 2.0f;

    chassis_mode = CHASSIS_ZERO_FORCE;
}

void Chassis_update(const Chassis_cmd_t *cmd, Chassis_output_t *output)
{
    if (cmd == NULL || output == NULL) return;

    memset(output, 0, sizeof(Chassis_output_t));

    if (chassis_mode == CHASSIS_ZERO_FORCE)
        return;

    float vx = cmd->vx;
    float vy = cmd->vy;
    float wz = cmd->omega * half_sum_LW;  /* omega * (L+W)/2 */

    /*
     * 麦轮逆运动学 (各轮线速度 mm/s)
     * FL(M1) = vx - vy - wz
     * RL(M2) = vx + vy - wz
     * FR(M3) = vx + vy + wz
     * RR(M4) = vx - vy + wz
     */
    float wheel_speed[4];
    wheel_speed[0] = vx - vy - wz;  /* M1: 左前 */
    wheel_speed[1] = vx + vy - wz;  /* M2: 左后 */
    wheel_speed[2] = vx + vy + wz;  /* M3: 右前 */
    wheel_speed[3] = vx - vy + wz;  /* M4: 右后 */

    /* 归一化: 找最大值，如果超出范围则等比缩放 */
    float max_speed = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        float abs_spd = (wheel_speed[i] > 0) ? wheel_speed[i] : -wheel_speed[i];
        if (abs_spd > max_speed)
            max_speed = abs_spd;
    }

    /* 将 mm/s 转换为 -1000~1000 比例值 */
    /* 假设 max_speed 对应 1000 (全速), 线性映射 */
    /* 如果 max_speed 为 0 则输出全 0 */
    if (max_speed > 0.01f)
    {
        /* 简单线性映射: speed_ratio = wheel_speed / wheel_circumference_per_sec * 1000 */
        /* 轮周长 = 2 * pi * R, 额定转速对应的线速度 */
        /* 这里先做简单映射: 直接用 vx=1000 对应 speed=1000 */
        float scale = 1.0f;

        /* 如果最大轮速超过 SPEED_MAX, 等比缩放 */
        if (max_speed > (float)SPEED_MAX)
            scale = (float)SPEED_MAX / max_speed;

        for (int i = 0; i < 4; i++)
        {
            int16_t spd = (int16_t)(wheel_speed[i] * scale);
            if (spd > SPEED_MAX) spd = SPEED_MAX;
            if (spd < SPEED_MIN) spd = SPEED_MIN;
            output->speed[i] = spd;
        }
    }
}

void Chassis_set_mode(Chassis_mode_e mode)
{
    chassis_mode = mode;
}

Chassis_mode_e Chassis_get_mode(void)
{
    return chassis_mode;
}
