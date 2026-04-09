/**
 * @file    robot_task.c
 * @brief   任务创建中枢 + 各任务函数实现
 */
#include "robot_task.h"
#include "error_handler.h"

/* ================= 任务句柄 ================= */

osThreadId_t motor_task_handle;
osThreadId_t sensor_task_handle;
osThreadId_t chassis_task_handle;
osThreadId_t decision_task_handle;
osThreadId_t test_task_handle;

/* ================= 队列句柄 ================= */

QueueHandle_t Chassis_cmd_queue;
QueueHandle_t Motor_cmd_queue;

/* ================= 全局变量 ================= */

Uart_instance_t *test_uart = NULL;

/* ================= 任务函数声明 ================= */

static void Motor_task(void *argument);
static void Sensor_task(void *argument);
static void Chassis_task(void *argument);
static void Decision_making_task(void *argument);

/* ================= 初始化 ================= */

void Robot_task_init(void)
{
    /* === BSP 初始化 === */
    DWT_Init(CPU_FREQ_MHZ);
    test_uart = Uart_register(&huart1, NULL);
    error_system_init(test_uart);
    Can_init();

    /* === 创建队列 (必须在任务之前) === */
    Chassis_cmd_queue = xQueueCreate(1, sizeof(Chassis_cmd_t));
    Motor_cmd_queue   = xQueueCreate(1, sizeof(Chassis_output_t));

    /* === 创建任务 === */

    /* motor_task: 最高优先级, 接收轮速→I2C 发送 */
    const osThreadAttr_t motor_attr = {
        .name = "motor_task",
        .priority = (osPriority_t)osPriorityAboveNormal,
        .stack_size = 512 * 4
    };
    motor_task_handle = osThreadNew(Motor_task, NULL, &motor_attr);

    /* sensor_task: 采集 IMU + 超声波 */
    const osThreadAttr_t sensor_attr = {
        .name = "sensor_task",
        .priority = (osPriority_t)osPriorityNormal,
        .stack_size = 512 * 4
    };
    sensor_task_handle = osThreadNew(Sensor_task, NULL, &sensor_attr);

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
    const osThreadAttr_t test_attr = {
        .name = "test_task",
        .priority = (osPriority_t)osPriorityBelowNormal,
        .stack_size = 256 * 4
    };
    test_task_handle = osThreadNew(Test_task, NULL, &test_attr);
}

/* ================= Motor Task ================= */

static void Motor_task(void *argument)
{
    (void)argument;

    /* 初始化电机驱动 */
    Yabo_motor_init_config_t motor_cfg = {
        .i2c_handle        = &hi2c2,
        .motor_type        = MOTOR_TYPE,
        .encoder_ppr       = MOTOR_ENCODER_PPR,
        .gear_ratio        = MOTOR_GEAR_RATIO,
        .wheel_diameter_mm = MOTOR_WHEEL_DIAMETER_MM,
        .deadzone          = MOTOR_DEADZONE,
    };
    Yabo_motor_init(&motor_cfg);

    Chassis_output_t motor_output;

    for (;;)
    {
        /* 等待底盘解算结果 */
        if (xQueueReceive(Motor_cmd_queue, &motor_output, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            Yabo_motor_set_speed(motor_output.speed);
        }

        /* 读取编码器反馈 */
        Yabo_motor_update();
    }
}

/* ================= Sensor Task ================= */

static void Sensor_task(void *argument)
{
    (void)argument;

    /* 初始化传感器 */
    DM_IMU_init_config_t imu_cfg = {
        .fdcan_handle = &hfdcan1,
        .can_id       = DM_IMU_CAN_ID,
        .mst_id       = DM_IMU_MST_ID,
        .mode         = DM_IMU_MODE_ACTIVE,
        .interval_ms  = DM_IMU_INTERVAL_MS,
    };
    Sensor_init(&imu_cfg);

    TickType_t *PreviousWakeTime = xTaskGetTickCount();

    for (;;)
    {
        Sensor_update();
        vTaskDelayUntil(&PreviousWakeTime, pdMS_TO_TICKS(10));
    }
}

/* ================= Chassis Task ================= */

static void Chassis_task(void *argument)
{
    (void)argument;

    /* 初始化底盘 */
    Chassis_params_t chassis_params = {
        .wheel_radius_mm = CHASSIS_WHEEL_RADIUS_MM,
        .wheel_base_mm   = CHASSIS_WHEEL_BASE_MM,
        .track_width_mm  = CHASSIS_TRACK_WIDTH_MM,
    };
    Chassis_init(&chassis_params);
    Chassis_set_mode(CHASSIS_NO_FOLLOW);

    Chassis_cmd_t cmd;
    Chassis_output_t output;

    for (;;)
    {
        /* 接收底盘速度指令 */
        if (xQueueReceive(Chassis_cmd_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            Chassis_update(&cmd, &output);
            xQueueOverwrite(Motor_cmd_queue, &output);
        }
    }
}

/* ================= Decision Making Task ================= */

static void Decision_making_task(void *argument)
{
    (void)argument;

    /* 初始化 PS2 和控制逻辑 */
    PS2_init();
    PS2_Control_init();

    for (;;)
    {
        /* 读取 PS2 数据 */
        PS2_update();

        /* 更新控制逻辑 */
        PS2_Control_update();

        /* 发布底盘指令 */
        const Chassis_cmd_t *cmd = PS2_Control_get_cmd();
        xQueueOverwrite(Chassis_cmd_queue, cmd);

        /* 喂 IWDG */
        IWDG_Feed();

        osDelay(10);
    }
}

/* ================= Stack Overflow Hook ================= */
/* 注意: vApplicationStackOverflowHook 已在 app_freertos.c 中定义 */
