/**
 * @file    dm_imu.h
 * @brief   达妙 DM-IMU-L1 驱动 (CAN 通信)
 * @note    内置 BMI088 + EKF，直接输出欧拉角/四元数/加速度/角速度
 *          CAN 协议: 请求帧 STD DLC=8, 应答帧 STD DLC=8
 *          映射转换: uint_to_float(val, min, max, bits)
 */
#ifndef _DM_IMU_H
#define _DM_IMU_H

#include "bsp_can.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief DM-IMU CAN 寄存器 ID
 */
#define DM_IMU_REG_ACCEL      0x01  /**< 加速度 (R) */
#define DM_IMU_REG_GYRO       0x02  /**< 角速度 (R) */
#define DM_IMU_REG_EULER      0x03  /**< 欧拉角 (R) */
#define DM_IMU_REG_QUAT       0x04  /**< 四元数 (R) */
#define DM_IMU_REG_ZERO       0x05  /**< 角度置零 (W) */
#define DM_IMU_REG_INTERVAL   0x0A  /**< 主动发送间隔 (RW) */
#define DM_IMU_REG_MODE       0x0B  /**< 主被动模式 (RW) */
#define DM_IMU_REG_SAVE       0xFE  /**< 保存参数 (W) */

/**
 * @brief DM-IMU 通信模式
 */
typedef enum {
    DM_IMU_MODE_PASSIVE = 0,  /**< 应答模式 (MCU 请求后回复) */
    DM_IMU_MODE_ACTIVE  = 1,  /**< 主动模式 (模块定时推送) */
} DM_IMU_mode_e;

/**
 * @brief 三轴数据
 */
typedef struct {
    float x;
    float y;
    float z;
} DM_IMU_vec3_t;

/**
 * @brief 四元数
 */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} DM_IMU_quat_t;

/**
 * @brief 欧拉角
 */
typedef struct {
    float pitch;   /**< ±90° */
    float yaw;     /**< ±180° */
    float roll;    /**< ±180° */
} DM_IMU_euler_t;

/**
 * @brief DM-IMU 初始化配置
 */
typedef struct {
    FDCAN_HandleTypeDef *fdcan_handle;  /**< FDCAN 句柄 */
    uint32_t can_id;                    /**< 发送帧 ID (模块的 CAN_ID) */
    uint32_t mst_id;                    /**< 应答帧 ID (模块回复的 MST_ID) */
    DM_IMU_mode_e mode;                 /**< 通信模式 */
    uint16_t interval_ms;               /**< 主动模式发送间隔 (ms) */
} DM_IMU_init_config_t;

/**
 * @brief DM-IMU 实例
 */
typedef struct {
    Can_controller_t *can_dev;          /**< CAN 设备实例 */
    uint32_t can_id;                    /**< 发送 ID */
    uint32_t mst_id;                    /**< 接收 ID */

    /* 传感器数据 */
    DM_IMU_vec3_t  accel;               /**< 加速度 (m/s²) */
    DM_IMU_vec3_t  gyro;                /**< 角速度 (rad/s) */
    DM_IMU_euler_t euler;               /**< 欧拉角 (°) */
    DM_IMU_quat_t  quaternion;          /**< 四元数 */
    float          temperature;         /**< 温度 (°C) */

    /* 状态 */
    bool           connected;
    uint32_t       update_count;        /**< 更新计数 */
} DM_IMU_instance_t;

/**
 * @brief 初始化 DM-IMU
 */
void DM_IMU_init(DM_IMU_init_config_t *cfg);

/**
 * @brief 解析 CAN 回调数据 (在 CAN 接收回调中调用)
 * @param can_data 8 字节 CAN 数据
 */
void DM_IMU_update(uint8_t *can_data);

/**
 * @brief 请求数据 (应答模式下使用)
 * @param reg 寄存器 ID (DM_IMU_REG_ACCEL/GYRO/EULER/QUAT)
 */
void DM_IMU_request(uint8_t reg);

/**
 * @brief 角度置零
 */
void DM_IMU_zero_angle(void);

/* ===== 数据获取接口 ===== */

const DM_IMU_euler_t *DM_IMU_get_euler(void);
const DM_IMU_vec3_t  *DM_IMU_get_accel(void);
const DM_IMU_vec3_t  *DM_IMU_get_gyro(void);
const DM_IMU_quat_t  *DM_IMU_get_quaternion(void);
float DM_IMU_get_temperature(void);
const DM_IMU_instance_t *DM_IMU_get_instance(void);

#endif /* _DM_IMU_H */
