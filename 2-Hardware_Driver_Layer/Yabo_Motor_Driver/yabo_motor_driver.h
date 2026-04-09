/**
 * @file    yabo_motor_driver.h
 * @brief   亚博智能四路电机驱动板 (I2C 通信, 地址 0x26)
 * @note    寄存器协议:
 *          写: 0x01(电机类型), 0x06(速度×4), 0x07(PWM×4)
 *          读: 0x08(电量), 0x10-0x13(实时编码器), 0x20-0x27(总编码器)
 */
#ifndef _YABO_MOTOR_DRIVER_H
#define _YABO_MOTOR_DRIVER_H

#include "bsp_iic.h"
#include <stdint.h>
#include <stdbool.h>

#define YABO_I2C_ADDR       0x26   /**< 7-bit I2C 地址 */
#define YABO_MOTOR_COUNT    4

/* 寄存器地址 */
#define YABO_REG_MOTOR_TYPE     0x01
#define YABO_REG_DEADZONE       0x02
#define YABO_REG_ENCODER_PPR    0x03
#define YABO_REG_GEAR_RATIO     0x04
#define YABO_REG_WHEEL_DIAM     0x05
#define YABO_REG_SPEED          0x06
#define YABO_REG_PWM            0x07
#define YABO_REG_BATTERY        0x08
#define YABO_REG_ENCODER_RT_M1  0x10
#define YABO_REG_ENCODER_RT_M2  0x11
#define YABO_REG_ENCODER_RT_M3  0x12
#define YABO_REG_ENCODER_RT_M4  0x13
#define YABO_REG_ENCODER_TOT_M1 0x20

/**
 * @brief 电机类型枚举
 */
typedef enum {
    YABO_MOTOR_520         = 1,
    YABO_MOTOR_310         = 2,
    YABO_MOTOR_TT_ENCODER  = 3,
    YABO_MOTOR_TT_NO_ENC   = 4,
} Yabo_motor_type_e;

/**
 * @brief 初始化配置
 */
typedef struct {
    I2C_HandleTypeDef *i2c_handle;     /**< I2C 句柄 */
    Yabo_motor_type_e motor_type;      /**< 电机类型 */
    uint16_t encoder_ppr;              /**< 编码器磁环线数 */
    uint16_t gear_ratio;               /**< 减速比 */
    float    wheel_diameter_mm;        /**< 轮子直径 (mm) */
    uint16_t deadzone;                 /**< 死区 (0-3600) */
} Yabo_motor_init_config_t;

/**
 * @brief 电机驱动实例
 */
typedef struct {
    IIC_instance_t *iic;               /**< I2C 设备实例 */

    /* 编码器反馈 */
    int16_t  encoder_rt[4];            /**< 实时编码器脉冲 (10ms) */
    int32_t  encoder_total[4];         /**< 总编码器脉冲 */

    /* 电池 */
    float    battery_voltage;          /**< 电池电压 (V) */

    /* 状态 */
    bool     connected;                /**< 通信是否正常 */
} Yabo_motor_instance_t;

/**
 * @brief 初始化电机驱动 (配置电机类型、编码器等参数)
 */
void Yabo_motor_init(Yabo_motor_init_config_t *cfg);

/**
 * @brief 读取编码器和电池数据
 */
void Yabo_motor_update(void);

/**
 * @brief 设置 4 路速度 (带编码器, -1000~1000)
 * @param speed 4 个电机的目标速度
 */
void Yabo_motor_set_speed(int16_t speed[4]);

/**
 * @brief 设置 4 路 PWM (-3600~3600)
 * @param pwm 4 个电机的 PWM 值
 */
void Yabo_motor_set_pwm(int16_t pwm[4]);

/**
 * @brief 获取电池电压
 */
float Yabo_motor_get_battery(void);

/**
 * @brief 获取电机实例指针
 */
const Yabo_motor_instance_t *Yabo_motor_get_instance(void);

#endif /* _YABO_MOTOR_DRIVER_H */
