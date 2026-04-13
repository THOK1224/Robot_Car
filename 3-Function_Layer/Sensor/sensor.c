/**
 * @file    sensor.c
 * @brief   传感器数据聚合模块实现
 */
#include "sensor.h"
#include "robot_definitions.h"
#include <string.h>

#include "bsp_usart.h"
#include "robot_task.h"
/* ================= 私有变量 ================= */

static Sensor_data_t sensor_data;

/* ================= 公开接口 ================= */

void Sensor_init(void)
{
    memset(&sensor_data, 0, sizeof(sensor_data));

    /* 初始化超声波 */
    Ultrasonic_init();

    // /* 初始化传感器 */
    // DM_IMU_init_config_t imu_cfg = {
    //     .fdcan_handle = &hfdcan1,
    //     .can_id       = DM_IMU_CAN_ID,
    //     .mst_id       = DM_IMU_MST_ID,
    //     .mode         = DM_IMU_MODE_ACTIVE,
    //     .interval_ms  = DM_IMU_INTERVAL_MS,
    // };
    // /* 初始化 IMU */
    // DM_IMU_init(&imu_cfg);
}

void Sensor_update(void)
{
    /* === 超声波 === */
    Ultrasonic_update();

    sensor_data.distance_cm     = Ultrasonic_get_distance();
    sensor_data.raw_distance_cm = Ultrasonic_get_raw_distance();
    // Uart_printf(test_uart, "Distance: %.2f cm (raw: %.2f cm)\r\n", sensor_data.distance_cm, sensor_data.raw_distance_cm);

    /* === IMU (CAN 回调自动更新, 这里只做数据拷贝) === */
    // const DM_IMU_instance_t *imu = DM_IMU_get_instance();
    // if (imu != NULL)
    // {
    //     sensor_data.euler           = imu->euler;
    //     sensor_data.accel           = imu->accel;
    //     sensor_data.gyro            = imu->gyro;
    //     sensor_data.quaternion      = imu->quaternion;
    //     sensor_data.imu_temperature = imu->temperature;
    //     sensor_data.imu_connected   = imu->connected;
    // }
}

const DM_IMU_euler_t *Sensor_get_euler(void)
{
    return &sensor_data.euler;
}

float Sensor_get_distance(void)
{
    return sensor_data.distance_cm;
}

const Sensor_data_t *Sensor_get_data(void)
{
    return &sensor_data;
}
