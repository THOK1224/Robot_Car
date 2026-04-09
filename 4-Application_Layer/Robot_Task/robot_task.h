/**
 * @file    robot_task.h
 * @brief   任务创建中枢
 */
#ifndef _ROBOT_TASK_H
#define _ROBOT_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"

#include "bsp_usart.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_wdg.h"

#include "chassis.h"
#include "ps2_control.h"
#include "sensor.h"
#include "yabo_motor_driver.h"
#include "dm_imu.h"
#include "robot_definitions.h"
#include "test_task.h"

/* 任务句柄 */
extern osThreadId_t motor_task_handle;
extern osThreadId_t sensor_task_handle;
extern osThreadId_t chassis_task_handle;
extern osThreadId_t decision_task_handle;
extern osThreadId_t test_task_handle;

/* 队列句柄 */
extern QueueHandle_t Chassis_cmd_queue;
extern QueueHandle_t Motor_cmd_queue;

/* 全局串口实例 */
extern Uart_instance_t *test_uart;

/**
 * @brief 初始化并创建所有任务
 */
void Robot_task_init(void);

#endif /* _ROBOT_TASK_H */