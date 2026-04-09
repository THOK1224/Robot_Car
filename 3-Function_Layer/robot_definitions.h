/**
 * @file    robot_definitions.h
 * @brief   全局定义和配置常量
 */
#ifndef _ROBOT_DEFINITIONS_H
#define _ROBOT_DEFINITIONS_H

/* ================= 底盘参数 ================= */
#define CHASSIS_WHEEL_RADIUS_MM    97.0f
#define CHASSIS_WHEEL_BASE_MM      250.0f
#define CHASSIS_TRACK_WIDTH_MM     250.0f

/* ================= 电机参数 ================= */
#define MOTOR_TYPE                 YABO_MOTOR_520
#define MOTOR_ENCODER_PPR          11
#define MOTOR_GEAR_RATIO           30
#define MOTOR_WHEEL_DIAMETER_MM    (CHASSIS_WHEEL_RADIUS_MM * 2.0f)
#define MOTOR_DEADZONE             1600

/* ================= DM-IMU 参数 ================= */
#define DM_IMU_CAN_ID              0x01    /* 模块 CAN ID (根据实际设置修改) */
#define DM_IMU_MST_ID              0x11    /* 应答帧 ID (根据实际设置修改) */
#define DM_IMU_INTERVAL_MS         10      /* 主动模式间隔 ms */

/* ================= DWT 时钟 ================= */
#define CPU_FREQ_MHZ               170

#endif /* _ROBOT_DEFINITIONS_H */
