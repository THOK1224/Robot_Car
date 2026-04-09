/**
 * @file    sensor.h
 * @brief   传感器数据聚合模块 (IMU + 超声波)
 */
#ifndef _SENSOR_H
#define _SENSOR_H

#include "dm_imu.h"
#include "ultrasonic.h"

/**
 * @brief 聚合传感器数据
 */
typedef struct {
    /* IMU */
    DM_IMU_euler_t euler;
    DM_IMU_vec3_t  accel;
    DM_IMU_vec3_t  gyro;
    DM_IMU_quat_t  quaternion;
    float          imu_temperature;
    bool           imu_connected;

    /* 超声波 */
    float          distance_cm;
    float          raw_distance_cm;
} Sensor_data_t;

/**
 * @brief 初始化传感器模块
 * @param imu_cfg DM-IMU 初始化配置 (可 NULL 跳过)
 */
void Sensor_init(DM_IMU_init_config_t *imu_cfg);

/**
 * @brief 更新传感器数据 (周期调用)
 * @note  IMU 数据通过 CAN 回调自动更新，此函数主要处理超声波
 */
void Sensor_update(void);

/**
 * @brief 获取 IMU 数据
 */
const DM_IMU_euler_t *Sensor_get_euler(void);

/**
 * @brief 获取超声波距离 (cm)
 */
float Sensor_get_distance(void);

/**
 * @brief 获取聚合数据
 */
const Sensor_data_t *Sensor_get_data(void);

#endif /* _SENSOR_H */
