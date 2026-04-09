/**
 * @file    yabo_motor_driver.c
 * @brief   亚博智能四路电机驱动板实现
 * @note    I2C 地址 0x26, 速度寄存器 0x06 大端序 int16_t×4
 *          电机映射: M1=左前, M2=左后, M3=右前, M4=右后
 */
#include "yabo_motor_driver.h"
#include <string.h>

/* ================= 私有变量 ================= */

static Yabo_motor_instance_t yabo_inst;

#define YABO_I2C_TIMEOUT  50  /* ms */

/* ================= 内部函数 ================= */

/**
 * @brief 写寄存器
 */
static HAL_StatusTypeDef _write_reg(uint8_t reg, uint8_t *data, uint16_t len)
{
    return IIC_write_reg(yabo_inst.iic, reg, data, len, YABO_I2C_TIMEOUT);
}

/**
 * @brief 读寄存器
 */
static HAL_StatusTypeDef _read_reg(uint8_t reg, uint8_t *data, uint16_t len)
{
    return IIC_read_reg(yabo_inst.iic, reg, data, len, YABO_I2C_TIMEOUT);
}

/**
 * @brief 打包 4 路 int16_t 为大端序字节流
 */
static void _pack_int16x4_be(int16_t val[4], uint8_t *buf)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        buf[i * 2]     = (uint8_t)(val[i] >> 8);    /* 高字节 */
        buf[i * 2 + 1] = (uint8_t)(val[i] & 0xFF);  /* 低字节 */
    }
}

/**
 * @brief 解析实时编码器 (单个 int16_t, 大端序)
 */
static int16_t _parse_encoder_int16(uint8_t *data)
{
    return (int16_t)((data[0] << 8) | data[1]);
}

/**
 * @brief 读取实时编码器 (4 路)
 */
static void _read_realtime_encoder(void)
{
    uint8_t buf[2];

    for (uint8_t i = 0; i < 4; i++)
    {
        if (_read_reg(YABO_REG_ENCODER_RT_M1 + i, buf, 2) == HAL_OK)
        {
            yabo_inst.encoder_rt[i] = _parse_encoder_int16(buf);
        }
    }
}

/**
 * @brief 读取总编码器 (4 路, 每路 4 字节 = 高16+低16)
 */
static void _read_total_encoder(void)
{
    uint8_t buf[4];

    for (uint8_t i = 0; i < 4; i++)
    {
        if (_read_reg(YABO_REG_ENCODER_TOT_M1 + i * 2, buf, 4) == HAL_OK)
        {
            int16_t high = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t low  = (int16_t)((buf[2] << 8) | buf[3]);
            yabo_inst.encoder_total[i] = ((int32_t)high << 16) | (uint16_t)low;
        }
    }
}

/* ================= 公开接口 ================= */

void Yabo_motor_init(Yabo_motor_init_config_t *cfg)
{
    if (cfg == NULL) return;

    memset(&yabo_inst, 0, sizeof(yabo_inst));

    /* 注册 I2C 设备 */
    IIC_init_t iic_cfg = {
        .i2c_handle  = cfg->i2c_handle,
        .dev_addr    = YABO_I2C_ADDR,
        .mode        = IIC_MODE_POLLING,
        .context     = NULL,
        .rx_callback = NULL,
    };
    yabo_inst.iic = IIC_register(&iic_cfg);

    if (yabo_inst.iic == NULL) return;

    /* 配置电机类型 */
    uint8_t type = (uint8_t)cfg->motor_type;
    _write_reg(YABO_REG_MOTOR_TYPE, &type, 1);

    /* 配置死区 */
    if (cfg->deadzone > 0)
    {
        uint8_t dz[2] = {(cfg->deadzone >> 8) & 0xFF, cfg->deadzone & 0xFF};
        _write_reg(YABO_REG_DEADZONE, dz, 2);
    }

    /* 配置编码器线数 */
    if (cfg->encoder_ppr > 0)
    {
        uint8_t ppr[2] = {(cfg->encoder_ppr >> 8) & 0xFF, cfg->encoder_ppr & 0xFF};
        _write_reg(YABO_REG_ENCODER_PPR, ppr, 2);
    }

    /* 配置减速比 */
    if (cfg->gear_ratio > 0)
    {
        uint8_t gr[2] = {(cfg->gear_ratio >> 8) & 0xFF, cfg->gear_ratio & 0xFF};
        _write_reg(YABO_REG_GEAR_RATIO, gr, 2);
    }

    /* 配置轮径 (float, 小端序) */
    if (cfg->wheel_diameter_mm > 0.0f)
    {
        uint8_t wd[4];
        memcpy(wd, &cfg->wheel_diameter_mm, 4);
        _write_reg(YABO_REG_WHEEL_DIAM, wd, 4);
    }

    yabo_inst.connected = true;
}

void Yabo_motor_update(void)
{
    _read_realtime_encoder();
    _read_total_encoder();

    /* 读取电池电量 */
    uint8_t bat_buf[2];
    if (_read_reg(YABO_REG_BATTERY, bat_buf, 2) == HAL_OK)
    {
        uint16_t bat_raw = (bat_buf[0] << 8) | bat_buf[1];
        yabo_inst.battery_voltage = (float)bat_raw / 10.0f;
    }
}

void Yabo_motor_set_speed(int16_t speed[4])
{
    uint8_t buf[8];
    _pack_int16x4_be(speed, buf);
    _write_reg(YABO_REG_SPEED, buf, 8);
}

void Yabo_motor_set_pwm(int16_t pwm[4])
{
    uint8_t buf[8];
    _pack_int16x4_be(pwm, buf);
    _write_reg(YABO_REG_PWM, buf, 8);
}

float Yabo_motor_get_battery(void)
{
    return yabo_inst.battery_voltage;
}

const Yabo_motor_instance_t *Yabo_motor_get_instance(void)
{
    return &yabo_inst;
}
