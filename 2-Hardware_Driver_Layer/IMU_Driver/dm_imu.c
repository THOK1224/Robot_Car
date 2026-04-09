/**
 * @file    dm_imu.c
 * @brief   达妙 DM-IMU-L1 驱动实现 (CAN 通信)
 * @note    映射转换公式 (附录二):
 *          Accel: ±235.2 m/s², 16bit
 *          Gyro:  ±34.88 rad/s, 16bit
 *          Pitch: ±90°, 16bit
 *          Yaw/Roll: ±180°, 16bit
 *          Quaternion: ±1.0, 14bit
 */
#include "dm_imu.h"
#include <string.h>

/* ================= 私有变量 ================= */

static DM_IMU_instance_t imu_inst;

/* ================= 映射转换 ================= */

/**
 * @brief 无符号整数映射到浮点数
 * @param x_int  无符号整数值
 * @param x_min  浮点下限
 * @param x_max  浮点上限
 * @param bits   位数
 * @return 映射后的浮点值
 */
static float _uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    uint32_t span = (1u << bits) - 1u;
    float offset = x_min;
    float pct = (float)x_int / (float)span;
    return pct * (x_max - x_min) + offset;
}

/**
 * @brief 浮点数映射到无符号整数 (发送用)
 */
static uint16_t _float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    uint32_t span = (1u << bits) - 1u;
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) / (x_max - x_min) * (float)span);
}

/* ================= 数据解析 ================= */

/**
 * @brief 解析加速度帧 (RID=0x01)
 * @note  DATA[0]=01, DATA[1]=温度, DATA[2:7]=Acc XYZ (16bit, 小端)
 */
static void _parse_accel(uint8_t *data)
{
    imu_inst.temperature = (float)(int8_t)data[1];

    uint16_t ax_raw = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    uint16_t ay_raw = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    uint16_t az_raw = (uint16_t)data[6] | ((uint16_t)data[7] << 8);

    imu_inst.accel.x = _uint_to_float(ax_raw, -235.2f, 235.2f, 16);
    imu_inst.accel.y = _uint_to_float(ay_raw, -235.2f, 235.2f, 16);
    imu_inst.accel.z = _uint_to_float(az_raw, -235.2f, 235.2f, 16);
}

/**
 * @brief 解析角速度帧 (RID=0x02)
 * @note  DATA[0]=02, DATA[2:7]=Gyro XYZ (16bit, 小端)
 */
static void _parse_gyro(uint8_t *data)
{
    uint16_t gx_raw = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    uint16_t gy_raw = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    uint16_t gz_raw = (uint16_t)data[6] | ((uint16_t)data[7] << 8);

    imu_inst.gyro.x = _uint_to_float(gx_raw, -34.88f, 34.88f, 16);
    imu_inst.gyro.y = _uint_to_float(gy_raw, -34.88f, 34.88f, 16);
    imu_inst.gyro.z = _uint_to_float(gz_raw, -34.88f, 34.88f, 16);
}

/**
 * @brief 解析欧拉角帧 (RID=0x03)
 * @note  DATA[0]=03, DATA[2:3]=Pitch(±90°), DATA[4:5]=Yaw(±180°), DATA[6:7]=Roll(±180°)
 */
static void _parse_euler(uint8_t *data)
{
    uint16_t pitch_raw = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    uint16_t yaw_raw   = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    uint16_t roll_raw  = (uint16_t)data[6] | ((uint16_t)data[7] << 8);

    imu_inst.euler.pitch = _uint_to_float(pitch_raw, -90.0f, 90.0f, 16);
    imu_inst.euler.yaw   = _uint_to_float(yaw_raw, -180.0f, 180.0f, 16);
    imu_inst.euler.roll  = _uint_to_float(roll_raw, -180.0f, 180.0f, 16);
}

/**
 * @brief 解析四元数帧 (RID=0x04)
 * @note  DATA[0]=04, DATA[1:7]=WXYZ (14bit 位域紧凑排列)
 *        7 bytes = 56 bits → 4 × 14 bits
 *        W: bits[55:42], X: bits[41:28], Y: bits[27:14], Z: bits[13:0]
 */
static void _parse_quaternion(uint8_t *data)
{
    /* 拼接 7 字节为 56-bit 整数 */
    uint64_t raw = 0;
    for (int i = 1; i <= 7; i++)
    {
        raw = (raw << 8) | data[i];
    }

    uint16_t w_raw = (raw >> 42) & 0x3FFF;
    uint16_t x_raw = (raw >> 28) & 0x3FFF;
    uint16_t y_raw = (raw >> 14) & 0x3FFF;
    uint16_t z_raw = (raw >>  0) & 0x3FFF;

    imu_inst.quaternion.w = _uint_to_float(w_raw, -1.0f, 1.0f, 14);
    imu_inst.quaternion.x = _uint_to_float(x_raw, -1.0f, 1.0f, 14);
    imu_inst.quaternion.y = _uint_to_float(y_raw, -1.0f, 1.0f, 14);
    imu_inst.quaternion.z = _uint_to_float(z_raw, -1.0f, 1.0f, 14);
}

/* ================= CAN 回调 ================= */

/**
 * @brief CAN 接收回调 (由 bsp_can 注册)
 */
static void _can_rx_callback(Can_controller_t *can, void *context)
{
    (void)context;
    DM_IMU_update(can->rx_buffer);
}

/* ================= 公开接口 ================= */

void DM_IMU_init(DM_IMU_init_config_t *cfg)
{
    if (cfg == NULL) return;

    memset(&imu_inst, 0, sizeof(imu_inst));
    imu_inst.can_id = cfg->can_id;
    imu_inst.mst_id = cfg->mst_id;

    /* 注册 CAN 设备 (接收 MST_ID 的应答帧) */
    Can_init_t can_cfg = {
        .can_handle        = cfg->fdcan_handle,
        .can_id            = cfg->can_id,
        .tx_id             = 0,
        .rx_id             = cfg->mst_id,
        .context           = NULL,
        .receive_callback  = _can_rx_callback,
    };
    imu_inst.can_dev = Can_device_init(&can_cfg);

    /* 如果配置为主动模式，发送模式切换和间隔设置 */
    if (cfg->mode == DM_IMU_MODE_ACTIVE && imu_inst.can_dev != NULL)
    {
        /* 设置主动模式 */
        uint8_t mode_cmd[8] = {0xCC, DM_IMU_REG_MODE, 0x01, 0x00,
                                0x01, 0x00, 0x00, 0x00};
        Can_send_data(imu_inst.can_dev, mode_cmd);

        /* 设置发送间隔 */
        if (cfg->interval_ms > 0)
        {
            uint8_t interval_cmd[8] = {0xCC, DM_IMU_REG_INTERVAL, 0x01, 0x00,
                                        (uint8_t)(cfg->interval_ms & 0xFF),
                                        (uint8_t)((cfg->interval_ms >> 8) & 0xFF),
                                        0x00, 0x00};
            Can_send_data(imu_inst.can_dev, interval_cmd);
        }
    }

    imu_inst.connected = true;
}

void DM_IMU_update(uint8_t *can_data)
{
    if (can_data == NULL) return;

    uint8_t reg_id = can_data[0];

    switch (reg_id)
    {
    case DM_IMU_REG_ACCEL:
        _parse_accel(can_data);
        break;
    case DM_IMU_REG_GYRO:
        _parse_gyro(can_data);
        break;
    case DM_IMU_REG_EULER:
        _parse_euler(can_data);
        break;
    case DM_IMU_REG_QUAT:
        _parse_quaternion(can_data);
        break;
    default:
        break;
    }

    imu_inst.update_count++;
}

void DM_IMU_request(uint8_t reg)
{
    if (imu_inst.can_dev == NULL) return;

    /* 请求帧: [CC, RID, R(0x00), DD(0x00), 0, 0, 0, 0] */
    uint8_t req[8] = {0xCC, reg, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    Can_send_data(imu_inst.can_dev, req);
}

void DM_IMU_zero_angle(void)
{
    if (imu_inst.can_dev == NULL) return;

    uint8_t cmd[8] = {0xCC, DM_IMU_REG_ZERO, 0x01, 0x00,
                       0x01, 0x00, 0x00, 0x00};
    Can_send_data(imu_inst.can_dev, cmd);
}

const DM_IMU_euler_t *DM_IMU_get_euler(void)
{
    return &imu_inst.euler;
}

const DM_IMU_vec3_t *DM_IMU_get_accel(void)
{
    return &imu_inst.accel;
}

const DM_IMU_vec3_t *DM_IMU_get_gyro(void)
{
    return &imu_inst.gyro;
}

const DM_IMU_quat_t *DM_IMU_get_quaternion(void)
{
    return &imu_inst.quaternion;
}

float DM_IMU_get_temperature(void)
{
    return imu_inst.temperature;
}

const DM_IMU_instance_t *DM_IMU_get_instance(void)
{
    return &imu_inst;
}
