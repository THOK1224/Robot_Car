/**
 * @file    test_task.c
 * @brief   测试任务实现 (3 种测试模式)
 * @note    通过修改 current_test_mode 切换测试内容
 */
#include "test_task.h"
#include "bsp_usart.h"
#include "ps2_driver.h"
#include "sensor.h"
#include "yabo_motor_driver.h"
#include "dm_imu.h"
#include "ultrasonic.h"
#include "robot_definitions.h"

extern Uart_instance_t *test_uart;

/* === 当前测试模式 (修改此值切换) === */
static Test_mode_e current_test_mode = TEST_MODE_PS2;

/* ================= 测试函数 ================= */

/**
 * @brief PS2 手柄测试: 打印按键和摇杆值
 */
static void _test_ps2(void)
{
    const PS2_instance_t *ps2 = PS2_get_instance();

    Uart_printf(test_uart,
        "[PS2] conn=%d mode=0x%02X btn=0x%04X LX=%3d LY=%3d RX=%3d RY=%3d\r\n",
        ps2->connected, ps2->mode, ps2->buttons,
        ps2->lx, ps2->ly, ps2->rx, ps2->ry);
}

/**
 * @brief 传感器测试: 打印 IMU 欧拉角和超声波距离
 */
static void _test_sensor(void)
{
    const DM_IMU_euler_t *euler = DM_IMU_get_euler();
    float dist = Ultrasonic_get_distance();
    float raw_dist = Ultrasonic_get_raw_distance();

    Uart_printf(test_uart,
        "[IMU] P=%.1f Y=%.1f R=%.1f T=%.0f  [US] dist=%.1f raw=%.1f cm\r\n",
        euler->pitch, euler->yaw, euler->roll,
        DM_IMU_get_temperature(),
        dist, raw_dist);
}

/**
 * @brief 电机测试: 低速旋转, 读取编码器
 */
static void _test_motor(void)
{
    const Yabo_motor_instance_t *motor = Yabo_motor_get_instance();

    /* 固定低速 */
    int16_t test_speed[4] = {200, 200, 200, 200};
    Yabo_motor_set_speed(test_speed);
    Yabo_motor_update();

    Uart_printf(test_uart,
        "[MOT] enc=[%d,%d,%d,%d] bat=%.1fV\r\n",
        motor->encoder_rt[0], motor->encoder_rt[1],
        motor->encoder_rt[2], motor->encoder_rt[3],
        motor->battery_voltage);
}

/* ================= 任务入口 ================= */

void Test_task(void *argument)
{
    (void)argument;

    for (;;)
    {
        if (test_uart != NULL)
        {
            switch (current_test_mode)
            {
            case TEST_MODE_PS2:
                _test_ps2();
                break;
            case TEST_MODE_SENSOR:
                _test_sensor();
                break;
            case TEST_MODE_MOTOR:
                _test_motor();
                break;
            default:
                break;
            }
        }

        osDelay(10);
    }
}
