/**
 * @file    bsp_wdg.c
 * @brief   看门狗模块实现
 * @note    移植自 SYSU_Hero，增加 IWDG 硬件看门狗封装
 */
#include "bsp_wdg.h"
#include <string.h>
#include "iwdg.h"

/* ================= 软件看门狗 ================= */

static Watchdog_device_t *wdg_pool[WATCHDOG_MX_NUM] = {NULL};
static uint8_t wdg_idx = 0;

Watchdog_device_t *Watchdog_register(Watchdog_init_t *config) {
    if (wdg_idx >= WATCHDOG_MX_NUM) return NULL;

    Watchdog_device_t *inst = (Watchdog_device_t *)malloc(sizeof(Watchdog_device_t));
    if (inst == NULL) return NULL;

    memset(inst, 0, sizeof(Watchdog_device_t));
    inst->owner_id         = config->owner_id;
    inst->reload_count     = config->reload_count == 0 ? 100 : config->reload_count;
    inst->offline_callback = config->callback;
    inst->online_callback  = config->online_callback;
    strncpy(inst->name, config->name, sizeof(inst->name) - 1);

    inst->temp_count = inst->reload_count;
    inst->is_offline = 0;

    wdg_pool[wdg_idx++] = inst;
    return inst;
}

void Watchdog_feed(Watchdog_device_t *instance) {
    if (instance == NULL) return;

    instance->temp_count = instance->reload_count;

    if (instance->is_offline == 1) {
        instance->is_offline = 0;
        if (instance->online_callback)
            instance->online_callback(instance->owner_id);
    }
}

void Watchdog_control_all(void) {
    for (uint8_t i = 0; i < wdg_idx; ++i) {
        Watchdog_device_t *dog = wdg_pool[i];
        if (dog == NULL) continue;

        if (dog->temp_count > 0) {
            dog->temp_count--;
        } else {
            if (dog->is_offline == 0)
                dog->is_offline = 1;
            if (dog->offline_callback)
                dog->offline_callback(dog->owner_id);
            dog->temp_count = dog->reload_count;
        }
    }
}

uint8_t Watchdog_is_online(Watchdog_device_t *instance) {
    if (instance == NULL) return 0;
    return (instance->is_offline == 0) ? 1u : 0u;
}

/* ================= IWDG 硬件看门狗 ================= */

void IWDG_Feed(void) {
    HAL_IWDG_Refresh(&hiwdg);
}
