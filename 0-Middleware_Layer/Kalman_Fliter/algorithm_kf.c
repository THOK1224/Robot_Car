/**
 * @file    algorithm_kf.c
 * @brief   1D 卡尔曼滤波器实现
 */
#include <stddef.h>
#include "algorithm_kf.h"

void Kf_init(Kf_state_t *kf, float q, float r)
{
    if (kf == NULL) return;

    kf->x = 0.0f;
    kf->p = 1.0f;
    kf->q = q;
    kf->r = r;
    kf->k = 0.0f;
    kf->is_initialized = true;
}

float Kf_update(Kf_state_t *kf, float measurement)
{
    if (kf == NULL || !kf->is_initialized) return 0.0f;

    /* 预测: 1D 恒值模型，协方差增加 */
    kf->p += kf->q;

    /* 更新 */
    kf->k = kf->p / (kf->p + kf->r);
    kf->x += kf->k * (measurement - kf->x);
    kf->p *= (1.0f - kf->k);

    return kf->x;
}

void Kf_reset(Kf_state_t *kf, float value)
{
    if (kf == NULL) return;

    kf->x = value;
    kf->p = 1.0f;
    kf->k = 0.0f;
}
