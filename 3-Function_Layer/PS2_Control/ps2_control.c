/**
 * @file    ps2_control.c
 * @brief   PS2 手柄控制逻辑实现
 * @note    L1 按键循环切换底盘模式 (零力 → 平移 → 旋转)
 *          平移模式: lx → vy (左右), ly → vx (前后)
 *          旋转模式: lx → wz
 *          通过队列发送指令给底盘任务
 */

#include "ps2_driver.h"
#include "ps2_control.h"
#include "robot_task.h"

/* ================= 私有变量 ================= */

static const PS2_instance_t *ps2_inst;
static PS2_control_instance_t ctrl_inst;

/* 摇杆死区 */
#define STICK_DEADZONE  20
/* 摇杆中心值 */
#define STICK_CENTER    128

/* ================= 内部函数 ================= */

/**
 * @brief 摇杆值转换为归一化值 (-1.0 ~ 1.0)
 */
static float _stick_to_float(uint8_t raw)
{
    int16_t centered = (int16_t)(raw - STICK_CENTER);

    if (centered > -STICK_DEADZONE && centered < STICK_DEADZONE)
        return 0.0f;

    return (float)centered / 128.0f;
}

/* ================= 公开接口 ================= */

void PS2_Control_init(void) {
    ctrl_inst.mode = PS2_CTRL_MODE_IDLE;
    ctrl_inst.cmd.chassis_vx = 0.0f;
    ctrl_inst.cmd.chassis_vy = 0.0f;
    ctrl_inst.cmd.chassis_wz = 0.0f;
    ctrl_inst.cmd.chassis_mode = CHASSIS_ZERO_FORCE;
    ctrl_inst.speed_scale = 500.0f;  /* 默认最大 500 mm/s */

    ps2_inst = PS2_get_instance();
}

void PS2_Control_update(void) {
    /* PS2 未连接 → 零力 */
    if (!ps2_inst->connected) {
        ctrl_inst.cmd.chassis_vx = 0.0f;
        ctrl_inst.cmd.chassis_vy = 0.0f;
        ctrl_inst.cmd.chassis_wz = 0.0f;
        ctrl_inst.cmd.chassis_mode = CHASSIS_ZERO_FORCE;
        xQueueOverwrite(Chassis_cmd_queue, &ctrl_inst.cmd);
        return;
    }

    /* L1 按键循环切换底盘模式: 零力 → 平移 → 旋转 → 零力 */
    if (ps2_inst->buttons_edge & PS2_BTN_L1) 
        ctrl_inst.cmd.chassis_mode = (ctrl_inst.cmd.chassis_mode + 1) % 3;

    /* 清零速度 */
    ctrl_inst.cmd.chassis_vx = 0.0f;
    ctrl_inst.cmd.chassis_vy = 0.0f;
    ctrl_inst.cmd.chassis_wz = 0.0f;

    if (ctrl_inst.cmd.chassis_mode == CHASSIS_TRANSLATION) {
        /* 平移模式: lx → vy (左右), ly → vx (前后) */
        float lx_norm = _stick_to_float(ps2_inst->lx);
        float ly_norm = -_stick_to_float(ps2_inst->ly);  /* Y 轴反转: 向上为正 */
        ctrl_inst.cmd.chassis_vx = ly_norm * ctrl_inst.speed_scale;
        ctrl_inst.cmd.chassis_vy = lx_norm * ctrl_inst.speed_scale;
    } else if (ctrl_inst.cmd.chassis_mode == CHASSIS_ROTATE) {
        /* 旋转模式: lx → wz */
        float lx_norm = _stick_to_float(ps2_inst->lx);
        ctrl_inst.cmd.chassis_wz = lx_norm * 3.14f;  /* 最大约 π rad/s */
    }

    /* 发送到底盘队列 */
    xQueueOverwrite(Chassis_cmd_queue, &ctrl_inst.cmd);
}
