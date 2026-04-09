/**
 * @file    ultrasonic.c
 * @brief   超声波测距模块实现 (EXTI + TIM7)
 * @note    TIM7: PSC=169, 1µs/tick, Period=65535
 *          Echo: PC7 EXTI Rising/Falling
 *          Trig: PC8 Output
 */
#include "ultrasonic.h"
#include "tim.h"
#include "bsp_dwt.h"

/* 声速 / 2 = 0.017 cm/µs */
#define SOUND_SPEED_HALF_CM_PER_US  0.017f
#define US_MAX_DISTANCE_CM          400.0f
#define US_MIN_DISTANCE_CM          2.0f

/* ================= 私有变量 ================= */

static Ultrasonic_instance_t us_inst;

/* ================= 内部函数 ================= */

/**
 * @brief 将计数值转换为距离 (cm)
 */
static float _count_to_distance(uint32_t count)
{
    /* count * 1µs * 0.017 cm/µs */
    return (float)count * SOUND_SPEED_HALF_CM_PER_US;
}

/**
 * @brief 1D 卡尔曼滤波初始化
 */
static void _kalman_init(Kalman1D_t *kf)
{
    kf->x = 0.0f;
    kf->p = 1.0f;
    kf->q = 0.01f;     /* 过程噪声 (可调) */
    kf->r = 0.5f;      /* 测量噪声 (可调) */
    kf->k = 0.0f;
}

/**
 * @brief 1D 卡尔曼滤波更新
 */
static float _kalman_update(Kalman1D_t *kf, float measurement)
{
    /* 预测 */
    kf->p += kf->q;

    /* 更新 */
    kf->k = kf->p / (kf->p + kf->r);
    kf->x += kf->k * (measurement - kf->x);
    kf->p *= (1.0f - kf->k);

    return kf->x;
}

/* ================= 公开接口 ================= */

void Ultrasonic_init(void)
{
    us_inst.state         = US_STATE_IDLE;
    us_inst.echo_count    = 0;
    us_inst.raw_distance_cm = 0.0f;
    us_inst.distance_cm   = 0.0f;
    us_inst.data_ready    = false;
    us_inst.timeout_count = 0;

    _kalman_init(&us_inst.kalman);

    /* 启动 TIM7 自由计数 */
    HAL_TIM_Base_Start(&htim7);

    /* Trig 引脚初始拉低 */
    HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);
}

void Ultrasonic_trigger(void)
{
    if (us_inst.state != US_STATE_IDLE && us_inst.state != US_STATE_DONE
        && us_inst.state != US_STATE_TIMEOUT)
        return;

    us_inst.state      = US_STATE_WAIT_ECHO;
    us_inst.data_ready = false;

    /* 发送 >=10µs 的 Trig 高脉冲 */
    HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_SET);
    DWT_Delay_us(15);
    HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);
}

void Ultrasonic_update(void)
{
    if (us_inst.state == US_STATE_DONE)
    {
        float dist = _count_to_distance(us_inst.echo_count);

        /* 范围限制 */
        if (dist < US_MIN_DISTANCE_CM) dist = US_MIN_DISTANCE_CM;
        if (dist > US_MAX_DISTANCE_CM) dist = US_MAX_DISTANCE_CM;

        us_inst.raw_distance_cm = dist;
        us_inst.distance_cm = _kalman_update(&us_inst.kalman, dist);
        us_inst.state = US_STATE_IDLE;
    }
    else if (us_inst.state == US_STATE_WAIT_ECHO || us_inst.state == US_STATE_MEASURING)
    {
        /* 超时检测: TIM7 满量程 65535µs ≈ 65ms → 约 11m */
        us_inst.timeout_count++;
        if (us_inst.timeout_count > 100) /* 100 次无响应 */
        {
            us_inst.state = US_STATE_TIMEOUT;
            us_inst.timeout_count = 0;
        }
    }
}

float Ultrasonic_get_distance(void)
{
    return us_inst.distance_cm;
}

float Ultrasonic_get_raw_distance(void)
{
    return us_inst.raw_distance_cm;
}

/* ================= EXTI 回调 ================= */

void Ultrasonic_EXTI_callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != US_Echo_Pin) return;

    GPIO_PinState pin_state = HAL_GPIO_ReadPin(US_Echo_GPIO_Port, US_Echo_Pin);

    if (pin_state == GPIO_PIN_SET)
    {
        /* 上升沿: 开始计时 */
        if (us_inst.state == US_STATE_WAIT_ECHO)
        {
            __HAL_TIM_SET_COUNTER(&htim7, 0);
            us_inst.state = US_STATE_MEASURING;
            us_inst.timeout_count = 0;
        }
    }
    else
    {
        /* 下降沿: 停止计时 */
        if (us_inst.state == US_STATE_MEASURING)
        {
            us_inst.echo_count = __HAL_TIM_GET_COUNTER(&htim7);
            us_inst.state = US_STATE_DONE;
            us_inst.data_ready = true;
        }
    }
}

/**
 * @brief HAL GPIO EXTI 回调 (覆盖弱定义)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Ultrasonic_EXTI_callback(GPIO_Pin);
}
