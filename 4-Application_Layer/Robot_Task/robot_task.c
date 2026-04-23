/**
 * @file    robot_task.c
 * @brief   任务创建中枢 + 各任务函数实现
 */
#include "robot_task.h"
#include "error_handler.h"

/* ================= 任务句柄 ================= */

osThreadId_t wdg_task_handle;
osThreadId_t motor_task_handle;
osThreadId_t sensor_task_handle;
osThreadId_t chassis_task_handle;
osThreadId_t decision_task_handle;
osThreadId_t test_task_handle;

/* ================= 队列句柄 ================= */

QueueHandle_t Chassis_cmd_queue;

/* ================= 全局变量 ================= */

Uart_instance_t *test_uart = NULL;

/* ================= 任务函数声明 ================= */

static void Watchdog_control_task(void *argument);
static void Motor_task(void *argument);
static void Sensor_task(void *argument);
static void Chassis_task(void *argument);
static void Decision_making_task(void *argument);

/* ================= 初始化 ================= */

void Robot_task_init(void) {
    /* === BSP 初始化 === */
    DWT_Init();
    test_uart = Uart_register(&huart1, NULL);
    error_system_init(NULL);
    Can_init();

    /* === 创建队列 (必须在任务之前) === */
    Chassis_cmd_queue = xQueueCreate(1, sizeof(Chassis_cmd_t));

    /* === 创建任务 === */

    /* watchdog_task: 看门狗控制 */
    const osThreadAttr_t wdg_attr = {
        .name = "wdg_task",
        .priority = (osPriority_t)osPriorityAboveNormal,
        .stack_size = 512 * 4
    };
    wdg_task_handle = osThreadNew(Watchdog_control_task, NULL, &wdg_attr);

    /* motor_task: 最高优先级, 接收轮速→I2C 发送 */
    const osThreadAttr_t motor_attr = {
        .name = "motor_task",
        .priority = (osPriority_t)osPriorityAboveNormal,
        .stack_size = 512 * 4
    };
    motor_task_handle = osThreadNew(Motor_task, NULL, &motor_attr);

    /* sensor_task: 采集 IMU + 超声波 */
    // const osThreadAttr_t sensor_attr = {
    //     .name = "sensor_task",
    //     .priority = (osPriority_t)osPriorityNormal,
    //     .stack_size = 512 * 4
    // };
    // sensor_task_handle = osThreadNew(Sensor_task, NULL, &sensor_attr);

    /* chassis_task: 运动学解算 */
    const osThreadAttr_t chassis_attr = {
        .name = "chassis_task",
        .priority = (osPriority_t)osPriorityNormal,
        .stack_size = 512 * 4
    };
    chassis_task_handle = osThreadNew(Chassis_task, NULL, &chassis_attr);

    /* decision_making_task: PS2 → 底盘指令 */
    const osThreadAttr_t decision_attr = {
        .name = "decision_task",
        .priority = (osPriority_t)osPriorityNormal,
        .stack_size = 512 * 4
    };
    decision_task_handle = osThreadNew(Decision_making_task, NULL, &decision_attr);

    /* test_task: 调试 */
    // const osThreadAttr_t test_attr = {
    //     .name = "test_task",
    //     .priority = (osPriority_t)osPriorityBelowNormal,
    //     .stack_size = 256 * 4
    // };
    // test_task_handle = osThreadNew(Test_task, NULL, &test_attr);
}

/* ================= WDG Task ================= */

void Watchdog_control_task(void *argument) {
    (void)argument;
     // 任务主循环
    for (;;) {
        Watchdog_control_all();
        // 任务延时10ms，保持100Hz的运行频率
        osDelay(10);
    }

}

/* ================= Motor Task ================= */

static void Motor_task(void *argument) {
    (void)argument;

    /* 初始化电机驱动 */
    Motor_init();
    Yabo_motor_init();

    for (;;) {
        /* 驱动周期更新: 编码器反馈 + PWM 下发 */
        Yabo_motor_update();

        osDelay(10);
    }
}

/* ================= Sensor Task ================= */

static void Sensor_task(void *argument) {
    (void)argument;

    Sensor_init();

    TickType_t PreviousWakeTime = xTaskGetTickCount();

    for (;;) {
        Sensor_update();
        vTaskDelayUntil(&PreviousWakeTime, pdMS_TO_TICKS(10));
    }
}

/* ================= Chassis Task ================= */

static void Chassis_task(void *argument) {
    (void)argument;

    /* 初始化底盘 */
    Chassis_init();

    for (;;) {
        /* 底盘周期更新: 队列接收 → 解算 → PID → 发布 */
        Chassis_update();

        osDelay(10);
    }
}

/* ================= Decision Making Task ================= */

static void Decision_making_task(void *argument) {
    (void)argument;

    /* 初始化 PS2 和控制逻辑 */
    PS2_init();
    PS2_Control_init();

    for (;;) {
        /* 读取 PS2 数据 */
        PS2_update();
        /* 更新控制逻辑 */
        PS2_Control_update();
        /* 喂 IWDG */
        IWDG_Feed();

        osDelay(10);
    }
}

/* ================= Stack Overflow Hook ================= */
/* 注意: vApplicationStackOverflowHook 已在 app_freertos.c 中定义 */
