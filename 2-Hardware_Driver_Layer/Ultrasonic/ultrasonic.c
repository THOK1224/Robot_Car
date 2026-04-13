/**
 * @file    ultrasonic.c
 * @brief   超声波测距模块实现(EXTI + TIM7 + Kalman滤波)
 * @note    中断只更新计数值；任务更新函数负责单位换算、滤波和结果刷新。
 */
#include "ultrasonic.h"
#include "tim.h"
#include "bsp_dwt.h"

#include "bsp_usart.h"
#include "robot_task.h"

/* 声速的一半约为 0.017 cm/us，用于由回波时间直接换算距离。 */
#define SOUND_SPEED_HALF_CM_PER_US  0.017f
#define US_MAX_DISTANCE_CM          400.0f
#define US_MIN_DISTANCE_CM          2.0f

/* 卡尔曼滤波器参数 */
#define US_KF_Q  0.01f   /* 过程噪声 (越大跟踪越快) */
#define US_KF_R  0.5f    /* 测量噪声 (越大越平滑) */

/* ================= 私有变量 ================= */
static Ultrasonic_state_t us_state;

/* ================= 公开接口 ================= */

void Ultrasonic_init(void)
{
    /* 初始化状态 */
    us_state.echo_count      = 0;
    us_state.echo_captured   = false;
    us_state.raw_distance_cm = 0.0f;
    us_state.distance_cm     = 0.0f;
    us_state.measuring       = false;

    /* 初始化卡尔曼滤波器 */
    Kf_init(&us_state.kf, US_KF_Q, US_KF_R);

    /* 启动 TIM7 自由计数 */
    HAL_TIM_Base_Start(&htim7);

    /* Trig 引脚初始拉低，由 update() 负责发脉冲 */
    HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);
}

void Ultrasonic_update(void)
{
    /* 检查是否有新的 Echo 数据 */
    if (us_state.echo_captured) {
        us_state.echo_captured = false;

        /* 单位转换: TIM7 计数 -> cm */
        float dist = (float)us_state.echo_count * SOUND_SPEED_HALF_CM_PER_US;

        /* 范围限制 */
        if (dist < US_MIN_DISTANCE_CM) dist = US_MIN_DISTANCE_CM;
        if (dist > US_MAX_DISTANCE_CM) dist = US_MAX_DISTANCE_CM;

        /* 保存原始距离 */
        us_state.raw_distance_cm = dist;

        /* 卡尔曼滤波 */
        us_state.distance_cm = Kf_update(&us_state.kf, dist);
    }

    /* 不在测量过程中则发起新的触发脉冲 */
    if (!us_state.measuring) {
        /* HC-SR04 触发: Trig HIGH ≥10µs → Trig LOW */
        HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_SET);
        DWT_Delay_us(15);
        HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);
    }
}

float Ultrasonic_get_distance(void)
{
    return us_state.distance_cm;
}

float Ultrasonic_get_raw_distance(void)
{
    return us_state.raw_distance_cm;
}

/* ================= EXTI 回调 ================= */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != US_Echo_Pin) return;

    GPIO_PinState pin_state = HAL_GPIO_ReadPin(US_Echo_GPIO_Port, US_Echo_Pin);

    if (pin_state == GPIO_PIN_SET) {
        /* 上升沿开始计时 */
        __HAL_TIM_SET_COUNTER(&htim7, 0);
        us_state.measuring = true;
    } else {
        /* 下降沿停止计时，记录计数值，标记数据就绪 */
        if (us_state.measuring) {
            us_state.measuring = false;
            us_state.echo_count = __HAL_TIM_GET_COUNTER(&htim7);
            us_state.echo_captured = true;  /* 中断中只设置标志 */
        }
    }
}

