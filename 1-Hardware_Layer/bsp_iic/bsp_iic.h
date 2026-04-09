/**
 * @file    bsp_iic.h
 * @brief   I2C 底层驱动封装 (STM32G4 I2C2)
 * @note    支持轮询和 DMA 两种模式
 */
#ifndef _BSP_IIC_H
#define _BSP_IIC_H

#include "i2c.h"
#include <stdint.h>

#define IIC_MAX_COUNT 4

typedef struct IIC_instance IIC_instance_t;

/** @brief I2C 接收完成回调 */
typedef void (*IIC_callback_t)(IIC_instance_t *inst, void *context);

/**
 * @brief I2C 传输模式
 */
typedef enum {
    IIC_MODE_POLLING = 0,
    IIC_MODE_DMA     = 1,
} IIC_mode_e;

/**
 * @brief I2C 设备实例
 */
typedef struct IIC_instance
{
    I2C_HandleTypeDef *i2c_handle;  /**< I2C 句柄 */
    uint16_t dev_addr;              /**< 7-bit 设备地址 (左移1位后的值) */
    IIC_mode_e mode;                /**< 传输模式 */

    uint8_t  rx_buffer[32];         /**< 接收缓冲 */
    uint16_t rx_len;                /**< 期望接收长度 */

    void *context;                  /**< 用户上下文 */
    IIC_callback_t rx_callback;     /**< 接收完成回调 */
} IIC_instance_t;

/**
 * @brief I2C 初始化配置
 */
typedef struct
{
    I2C_HandleTypeDef *i2c_handle;
    uint16_t dev_addr;              /**< 7-bit 地址 (如 0x26) */
    IIC_mode_e mode;
    void *context;
    IIC_callback_t rx_callback;
} IIC_init_t;

/**
 * @brief 注册 I2C 设备实例
 */
IIC_instance_t *IIC_register(IIC_init_t *config);

/**
 * @brief 写寄存器 (阻塞)
 * @param inst    设备实例
 * @param reg     寄存器地址
 * @param data    数据指针
 * @param len     数据长度
 * @param timeout 超时 ms
 * @return HAL 状态
 */
HAL_StatusTypeDef IIC_write_reg(IIC_instance_t *inst, uint8_t reg,
                                 uint8_t *data, uint16_t len, uint32_t timeout);

/**
 * @brief 读寄存器 (阻塞)
 */
HAL_StatusTypeDef IIC_read_reg(IIC_instance_t *inst, uint8_t reg,
                                uint8_t *data, uint16_t len, uint32_t timeout);

/**
 * @brief DMA 方式写寄存器 (非阻塞)
 */
HAL_StatusTypeDef IIC_write_reg_dma(IIC_instance_t *inst, uint8_t reg,
                                     uint8_t *data, uint16_t len);

/**
 * @brief DMA 方式读寄存器 (非阻塞)
 */
HAL_StatusTypeDef IIC_read_reg_dma(IIC_instance_t *inst, uint8_t reg,
                                    uint16_t len);

#endif /* _BSP_IIC_H */
