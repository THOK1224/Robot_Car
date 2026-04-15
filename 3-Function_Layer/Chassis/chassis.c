/**
 * @file    chassis.c
 * @brief   麦克纳姆轮底盘运动学实现
 * @note    麦轮逆运动学:
 *          V_FL = vx - vy - (L+W)/2 * wz
 *          V_FR = vx + vy + (L+W)/2 * wz
 *          V_RL = vx + vy - (L+W)/2 * wz
 *          V_RR = vx - vy + (L+W)/2 * wz
 *          (L = wheel_base_mm, W = track_width_mm)
 *
 *          电机映射: M1=左前(FL), M2=右前(FR), M3=左后(RL), M4=右后(RR)
 */
#include "chassis.h"
#include "motor.h"
#include "message_center.h"
#include "robot_task.h"
#include "robot_definitions.h"
#include <string.h>

/* ================= 私有变量 ================= */

static Chassis_params_t chassis_params;
static Motor_instance_t *motor = NULL;
static Publisher_t *chassis_pub = NULL;

/* ================= 工具函数 ================= */

/**
 * @brief 麦克纳姆轮底盘运动学解算 (所有速度单位: mm/s)
 * @param cmd    底盘控制指令
 * @param output 解算输出结果
 */
static void Chassis_mecanum_kinematics(const Chassis_cmd_t *cmd, Chassis_output_t *output) {
    // 麦克纳姆轮布局（从俯视图看）：
    // 左前(M1)  右前(M2)
    // 左后(M3)  右后(M4)

    float half_LW = (chassis_params.wheel_base_mm + chassis_params.track_width_mm) / 2.0f;
    float rotate  = cmd->chassis_wz * half_LW;

    output->motor_speed[0] = -cmd->chassis_vx + cmd->chassis_vy + rotate;  // M1 左前
    output->motor_speed[1] = +cmd->chassis_vx + cmd->chassis_vy + rotate;  // M2 右前
    output->motor_speed[2] = +cmd->chassis_vx - cmd->chassis_vy + rotate;  // M3 左后
    output->motor_speed[3] = -cmd->chassis_vx - cmd->chassis_vy + rotate;  // M4 右后
}

/* ================= 公开接口 ================= */

void Chassis_init(void) {
    // 1. 底盘参数初始化
    chassis_params.wheel_radius_mm = CHASSIS_WHEEL_RADIUS_MM;
    chassis_params.wheel_base_mm   = CHASSIS_WHEEL_BASE_MM;
    chassis_params.track_width_mm  = CHASSIS_TRACK_WIDTH_MM;

    // 2. 获取电机实例 (电机初始化由 Yabo_motor_init 统一完成)
    motor = Motor_get_instance();

    // 3. 注册发布者：向电机驱动发送底盘输出
    chassis_pub = Pub_register("Chassis_output", sizeof(Chassis_output_t));
}

void Chassis_update(void) {
    // 1. 从底盘队列获取控制指令
    Chassis_cmd_t cmd;
    if (xQueueReceive(Chassis_cmd_queue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE)
        return;

    // 2. 运动学解算
    Chassis_output_t kin_output;
    memset(&kin_output, 0, sizeof(kin_output));

    switch (cmd.chassis_mode) {
    case CHASSIS_ZERO_FORCE:
        // 零力模式不解算，保持全零
        break;
    case CHASSIS_TRANSLATION:
    case CHASSIS_ROTATE:
        Chassis_mecanum_kinematics(&cmd, &kin_output);
        break;
    default:
        break;
    }

    // 3. 对每个电机进行 PID 计算
    for (int i = 0; i < 4; i++) {
        Motor_calculate(&motor[i], kin_output.motor_speed[i]);
    }

    // 4. 打包 Motor 输出并通过消息中心发布
    Chassis_output_t pwm_output;
    for (int i = 0; i < 4; i++) {
        pwm_output.motor_speed[i] = motor[i].output;
    }
    Pub_push_message(chassis_pub, &pwm_output);
}

