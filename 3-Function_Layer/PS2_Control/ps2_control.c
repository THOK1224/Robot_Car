/**
 * @file    ps2_control.c
 * @brief   PS2 手柄控制逻辑框架实现
 * @note    按键映射示例:
 *          左摇杆 X/Y → vx, vy
 *          右摇杆 X   → omega
 *          START       → 切换模式
 *          用户可根据需求修改映射
 */

#include "bsp_usart.h"
#include "ps2_driver.h"
#include "ps2_control.h"
#include "robot_task.h"

/* ================= 私有变量 ================= */

static PS2_control_instance_t ctrl_inst;

/* 摇杆死区 */
#define STICK_DEADZONE  20
/* 摇杆中心值 */
#define STICK_CENTER    128

/* ================= 内部函数 ================= */

/**
 * @brief 摇杆值转换为归一化值 (-1.0 ~ 1.0)
 */
static float _stick_to_float(uint8_t raw) {
    int16_t centered = (int16_t)raw - STICK_CENTER;

    if (centered > -STICK_DEADZONE && centered < STICK_DEADZONE)
        return 0.0f;

    return (float)centered / 128.0f;
}

/* ================= 公开接口 ================= */

void PS2_Control_init(void) {
    ctrl_inst.mode = PS2_CTRL_MODE_IDLE;
    ctrl_inst.cmd.vx = 0.0f;
    ctrl_inst.cmd.vy = 0.0f;
    ctrl_inst.cmd.omega = 0.0f;
    ctrl_inst.speed_scale = 500.0f;  /* 默认最大 500 mm/s, 可调 */
}

void PS2_Control_update(void) {
    const PS2_instance_t *ps2 = PS2_get_instance();

    /* PS2 未连接 → 零力 */
    if (!ps2->connected) {
        ctrl_inst.cmd.vx = 0.0f;
        ctrl_inst.cmd.vy = 0.0f;
        ctrl_inst.cmd.omega = 0.0f;
        ctrl_inst.mode = PS2_CTRL_MODE_IDLE;
        return;
    }

    /* START 键切换模式 */
    if (ps2->buttons_edge & PS2_BTN_START) {
        if (ctrl_inst.mode == PS2_CTRL_MODE_IDLE)
            ctrl_inst.mode = PS2_CTRL_MODE_MANUAL;
        else
            ctrl_inst.mode = PS2_CTRL_MODE_IDLE;
    }

    if (ctrl_inst.mode == PS2_CTRL_MODE_MANUAL) {
        /* 左摇杆 → 平移 */
        float ly_norm = -_stick_to_float(ps2->ly); /* Y 轴反转: 向上为正 */
        float lx_norm =  _stick_to_float(ps2->lx);

        /* 右摇杆 X → 旋转 */
        float rx_norm = _stick_to_float(ps2->rx);

        ctrl_inst.cmd.vx    = ly_norm * ctrl_inst.speed_scale;
        ctrl_inst.cmd.vy    = -lx_norm * ctrl_inst.speed_scale;
        ctrl_inst.cmd.omega = rx_norm * 3.14f;  /* 最大约 π rad/s */

        /* L1/R1 调速 (示例) */
        if (ps2->buttons_edge & PS2_BTN_R1 && ctrl_inst.speed_scale < 1000.0f)
            ctrl_inst.speed_scale += 100.0f;
        if (ps2->buttons_edge & PS2_BTN_L1 && ctrl_inst.speed_scale > 100.0f)
            ctrl_inst.speed_scale -= 100.0f;
    } else {
        ctrl_inst.cmd.vx = 0.0f;
        ctrl_inst.cmd.vy = 0.0f;
        ctrl_inst.cmd.omega = 0.0f;
    }

    /* 发布底盘指令 */
    xQueueOverwrite(Chassis_cmd_queue, &ctrl_inst.cmd);
}



