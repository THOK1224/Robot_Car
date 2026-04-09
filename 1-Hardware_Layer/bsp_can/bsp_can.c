/**
 * @file    bsp_can.c
 * @brief   FDCAN 底层驱动实现 (STM32G4)
 * @note    完全重写 Hero bsp_can，使用 FDCAN HAL API
 *          保留 LUT 快速查找架构
 */
#include "bsp_can.h"
#include <stdlib.h>
#include <string.h>

/* ================= LUT 快速查找表 ================= */

static Can_controller_t *fdcan1_rx_lut[CAN_FAST_LUT_SIZE] = {NULL};

/* 线性列表 (内存管理备份) */
static Can_controller_t *can_controller[CAN_MAX_COUNT] = {NULL};
static uint8_t can_ix = 0;

/* ================= 内部函数 ================= */

/**
 * @brief 配置 FDCAN1 全局过滤器 (接受所有标准帧到 FIFO0)
 */
static void Can_filter_config_global(void)
{
    /* 配置全局过滤器: 接受所有不匹配的标准帧到 FIFO0 */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                  FDCAN_ACCEPT_IN_RX_FIFO0,  /* 标准帧不匹配 → FIFO0 */
                                  FDCAN_REJECT,               /* 扩展帧: 拒绝 */
                                  FDCAN_FILTER_REMOTE,        /* 远程标准帧: 过滤 */
                                  FDCAN_FILTER_REMOTE);       /* 远程扩展帧: 过滤 */
}

/**
 * @brief 处理 FIFO 中的数据
 */
static void Can_fifo_process(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    Can_controller_t *target = NULL;

    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        /* 提取标准 ID (FDCAN 的 Identifier 字段) */
        uint32_t std_id = rx_header.Identifier;

        /* LUT 快速查找 */
        if (std_id < CAN_FAST_LUT_SIZE)
        {
            target = fdcan1_rx_lut[std_id];
        }

        if (target != NULL && target->receive_callback != NULL)
        {
            /* 快速拷贝 8 字节 */
            uint32_t *dst = (uint32_t *)target->rx_buffer;
            uint32_t *src = (uint32_t *)rx_data;
            dst[0] = src[0];
            dst[1] = src[1];

            target->receive_callback(target, target->context);
        }
        target = NULL;
    }
}

/* ================= 公开接口 ================= */

void Can_init(void)
{
    Can_filter_config_global();

    /* 启动 FDCAN */
    HAL_FDCAN_Start(&hfdcan1);

    /* 使能 FIFO0 新消息中断 */
    HAL_FDCAN_ActivateNotification(&hfdcan1,
                                    FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

Can_controller_t *Can_device_init(Can_init_t *can_config)
{
    if (can_ix >= CAN_MAX_COUNT || can_config == NULL) return NULL;

    Can_controller_t *dev = (Can_controller_t *)malloc(sizeof(Can_controller_t));
    if (dev == NULL) return NULL;
    memset(dev, 0, sizeof(Can_controller_t));

    dev->can_handle        = can_config->can_handle;
    dev->can_id            = can_config->can_id;
    dev->tx_id             = can_config->tx_id;
    dev->rx_id             = can_config->rx_id;
    dev->receive_callback  = can_config->receive_callback;
    dev->context           = can_config->context;

    /* 配置发送帧头 (Classic CAN, 8 bytes) */
    dev->tx_header.Identifier          = can_config->can_id;
    dev->tx_header.IdType              = FDCAN_STANDARD_ID;
    dev->tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    dev->tx_header.DataLength          = FDCAN_DLC_BYTES_8;
    dev->tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    dev->tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    dev->tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    dev->tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    dev->tx_header.MessageMarker       = 0;

    /* 注册到 LUT */
    if (can_config->rx_id < CAN_FAST_LUT_SIZE)
    {
        if (can_config->can_handle == &hfdcan1)
            fdcan1_rx_lut[can_config->rx_id] = dev;
    }

    can_controller[can_ix++] = dev;
    return dev;
}

uint8_t Can_send_data(Can_controller_t *can_dev, uint8_t *tx_buff)
{
    if (can_dev == NULL || tx_buff == NULL) return 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(can_dev->can_handle) == 0)
        return 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(can_dev->can_handle,
                                       &can_dev->tx_header,
                                       tx_buff) != HAL_OK)
        return 0;

    return 1;
}

/* ================= FDCAN 中断回调 ================= */

/**
 * @brief FDCAN FIFO0 接收回调
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        Can_fifo_process(hfdcan);
    }
}
