/**
 * @file    sensor.c
 * @brief   传感器数据聚合模块实现
 */
#include "sensor.h"
#include <string.h>

/* ================= 私有变量 ================= */

static Sensor_data_t sensor_data;
static uint8_t us_trigger_div = 0;  /* 分频器: 控制超声波触发频率 */

/* ================= 公开接口 ================= */

void Sensor_init(DM_IMU_init_config_t *imu_cfg)
{
    memset(&sensor_data, 0, sizeof(sensor_data));

    /* 初始化 IMU */
    if (imu_cfg != NULL)
    {
        DM_IMU_init(imu_cfg);
    }

    /* 初始化超声波 */
    Ultrasonic_init();
}

void Sensor_update(void)
{
    /* === 超声波 === */
    /* 每 5 次调用触发一次 (如 100Hz 调用 → 20Hz 测量) */
    us_trigger_div++;
    if (us_trigger_div >= 5)
    {
        us_trigger_div = 0;
        Ultrasonic_trigger();
    }
    Ultrasonic_update();

    sensor_data.distance_cm     = Ultrasonic_get_distance();
    sensor_data.raw_distance_cm = Ultrasonic_get_raw_distance();

    /* === IMU (CAN 回调自动更新, 这里只做数据拷贝) === */
    const DM_IMU_instance_t *imu = DM_IMU_get_instance();
    if (imu != NULL)
    {
        sensor_data.euler           = imu->euler;
        sensor_data.accel           = imu->accel;
        sensor_data.gyro            = imu->gyro;
        sensor_data.quaternion      = imu->quaternion;
        sensor_data.imu_temperature = imu->temperature;
        sensor_data.imu_connected   = imu->connected;
    }
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
