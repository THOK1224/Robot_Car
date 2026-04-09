/**
 * @file    test_task.h
 * @brief   测试任务 (3 种测试模式)
 */
#ifndef _TEST_TASK_H
#define _TEST_TASK_H

#include "cmsis_os.h"

/**
 * @brief 测试模式
 */
typedef enum {
    TEST_MODE_PS2    = 0,  /**< PS2 手柄数据打印 */
    TEST_MODE_SENSOR = 1,  /**< IMU + 超声波数据打印 */
    TEST_MODE_MOTOR  = 2,  /**< 电机驱动直接测试 */
} Test_mode_e;

/**
 * @brief 测试任务入口
 */
void Test_task(void *argument);

#endif /* _TEST_TASK_H */