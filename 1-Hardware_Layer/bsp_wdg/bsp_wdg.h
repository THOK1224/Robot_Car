/**
 * @file    bsp_wdg.h
 * @brief   看门狗模块：IWDG 硬件喂狗 + 软件设备离线监控
 * @note    移植自 SYSU_Hero，增加 IWDG 硬件看门狗封装
 */
#ifndef BSP_WDG_H
#define BSP_WDG_H

#include <stdint.h>
#include <stdlib.h>

#define WATCHDOG_MX_NUM 16

/* ================= 软件看门狗 (设备离线监控) ================= */

/** @brief 回调函数指针类型 */
typedef void (*wdg_callback_func)(void *);

/**
 * @brief 软件看门狗实例
 */
typedef struct
{
    uint16_t reload_count;         /**< 重载值 (单位: Watchdog_control_all 调用次数) */
    volatile uint16_t temp_count;  /**< 当前倒计数 */
    uint8_t  is_offline;           /**< 离线标志 */

    wdg_callback_func offline_callback; /**< 离线回调 */
    wdg_callback_func online_callback;  /**< 上线回调 */

    void *owner_id;                /**< 被监控对象 */
    char name[16];                 /**< 设备名称 */
} Watchdog_device_t;

/**
 * @brief 软件看门狗初始化配置
 */
typedef struct
{
    uint16_t reload_count;         /**< 超时值 (调用次数) */
    wdg_callback_func callback;    /**< 离线回调 */
    wdg_callback_func online_callback; /**< 上线回调 */
    void *owner_id;
    char name[16];
} Watchdog_init_t;

/**
 * @brief 注册软件看门狗实例
 */
Watchdog_device_t *Watchdog_register(Watchdog_init_t *config);

/**
 * @brief 喂狗 (ISR 安全)
 */
void Watchdog_feed(Watchdog_device_t *instance);

/**
 * @brief 全局轮询，放入 RTOS 任务中循环调用
 */
void Watchdog_control_all(void);

/**
 * @brief 检查设备是否在线
 */
uint8_t Watchdog_is_online(Watchdog_device_t *instance);

/* ================= IWDG 硬件看门狗 ================= */

/**
 * @brief 刷新 IWDG 硬件看门狗
 */
void IWDG_Feed(void);

#endif /* BSP_WDG_H */
