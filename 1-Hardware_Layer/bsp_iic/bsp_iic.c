/**
 * @file    bsp_iic.c
 * @brief   I2C 底层驱动实现 (STM32G4 I2C2)
 */
#include "bsp_iic.h"
#include <stdlib.h>
#include <string.h>

static IIC_instance_t *iic_pool[IIC_MAX_COUNT] = {NULL};
static uint8_t iic_idx = 0;

/* ================= 内部函数 ================= */

/**
 * @brief 根据 I2C 句柄和地址查找实例
 */
static IIC_instance_t *IIC_find_instance(I2C_HandleTypeDef *hi2c)
{
    for (uint8_t i = 0; i < iic_idx; i++)
    {
        if (iic_pool[i] != NULL && iic_pool[i]->i2c_handle == hi2c)
            return iic_pool[i];
    }
    return NULL;
}

/* ================= 公开接口 ================= */

IIC_instance_t *IIC_register(IIC_init_t *config)
{
    if (iic_idx >= IIC_MAX_COUNT || config == NULL) return NULL;

    IIC_instance_t *inst = (IIC_instance_t *)malloc(sizeof(IIC_instance_t));
    if (inst == NULL) return NULL;
    memset(inst, 0, sizeof(IIC_instance_t));

    inst->i2c_handle  = config->i2c_handle;
    inst->dev_addr    = config->dev_addr << 1; /* HAL 需要左移 1 位 */
    inst->mode        = config->mode;
    inst->context     = config->context;
    inst->rx_callback = config->rx_callback;

    iic_pool[iic_idx++] = inst;
    return inst;
}

HAL_StatusTypeDef IIC_write_reg(IIC_instance_t *inst, uint8_t reg,
                                 uint8_t *data, uint16_t len, uint32_t timeout)
{
    if (inst == NULL || data == NULL) return HAL_ERROR;

    return HAL_I2C_Mem_Write(inst->i2c_handle, inst->dev_addr,
                              reg, I2C_MEMADD_SIZE_8BIT,
                              data, len, timeout);
}

HAL_StatusTypeDef IIC_read_reg(IIC_instance_t *inst, uint8_t reg,
                                uint8_t *data, uint16_t len, uint32_t timeout)
{
    if (inst == NULL || data == NULL) return HAL_ERROR;

    return HAL_I2C_Mem_Read(inst->i2c_handle, inst->dev_addr,
                             reg, I2C_MEMADD_SIZE_8BIT,
                             data, len, timeout);
}

HAL_StatusTypeDef IIC_write_reg_dma(IIC_instance_t *inst, uint8_t reg,
                                     uint8_t *data, uint16_t len)
{
    if (inst == NULL || data == NULL) return HAL_ERROR;

    return HAL_I2C_Mem_Write_DMA(inst->i2c_handle, inst->dev_addr,
                                  reg, I2C_MEMADD_SIZE_8BIT,
                                  data, len);
}

HAL_StatusTypeDef IIC_read_reg_dma(IIC_instance_t *inst, uint8_t reg,
                                    uint16_t len)
{
    if (inst == NULL) return HAL_ERROR;

    inst->rx_len = len;
    return HAL_I2C_Mem_Read_DMA(inst->i2c_handle, inst->dev_addr,
                                 reg, I2C_MEMADD_SIZE_8BIT,
                                 inst->rx_buffer, len);
}

/* ================= DMA 回调 ================= */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    IIC_instance_t *inst = IIC_find_instance(hi2c);
    if (inst != NULL && inst->rx_callback != NULL)
    {
        inst->rx_callback(inst, inst->context);
    }
}
