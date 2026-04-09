/**
 * @file    bsp_can.h
 * @brief   FDCAN 底层驱动封装 (STM32G4 FDCAN)
 * @note    参考 SYSU_Hero bsp_can 架构，完全重写为 FDCAN API
 *          保留 LUT 快速查找优化
 */
#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "fdcan.h"
#include <stdint.h>

#define CAN_MAX_COUNT       16
#define CAN_FAST_LUT_SIZE   0x300

typedef struct CanController CanController_t;

/** @brief CAN 接收回调函数类型 */
typedef void (*ReceiveCallback_t)(CanController_t *, void *context);

/**
 * @brief CAN 控制器实例
 */
typedef struct CanController
{
    FDCAN_HandleTypeDef   *can_handle;  /**< FDCAN 句柄 */
    FDCAN_TxHeaderTypeDef  tx_header;   /**< 发送帧头配置 */
    uint32_t can_id;                    /**< 发送 StdId */
    uint32_t tx_id;                     /**< 设备编号 */
    uint32_t rx_id;                     /**< 接收过滤 StdId */
    uint8_t  rx_buffer[8];             /**< 接收数据缓冲 */

    void *context;                      /**< 用户上下文 */
    ReceiveCallback_t receive_callback; /**< 接收回调 */
} Can_controller_t;

/**
 * @brief CAN 初始化配置
 */
typedef struct
{
    FDCAN_HandleTypeDef *can_handle; /**< FDCAN 句柄 */
    uint32_t can_id;                 /**< 发送 StdId */
    uint32_t tx_id;                  /**< 设备编号 */
    uint32_t rx_id;                  /**< 接收 StdId */
    void *context;
    ReceiveCallback_t receive_callback;
} Can_init_t;

/**
 * @brief FDCAN 全局初始化 (过滤器 + 启动 + 中断使能)
 */
void Can_init(void);

/**
 * @brief 注册 CAN 设备
 * @return 设备实例指针, NULL 表示失败
 */
Can_controller_t *Can_device_init(Can_init_t *can_config);

/**
 * @brief 发送 8 字节数据
 * @return 1=成功, 0=失败
 */
uint8_t Can_send_data(Can_controller_t *can_dev, uint8_t *tx_buff);

#endif /* _BSP_CAN_H */
