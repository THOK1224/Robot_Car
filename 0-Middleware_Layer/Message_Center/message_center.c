/**
 * @file    message_center.c
 * @brief   消息中心实现 (发布-订阅模式)
 * @author  SYSU电控组
 * @note    移植自 SYSU_Hero
 */
#include "message_center.h"
#include "stdlib.h"
#include "string.h"

static SemaphoreHandle_t g_message_center_mutex = NULL;

static Publisher_t message_center = {
    .topic_name = "Message_Manager",
    .first_subs = NULL,
    .next_topic_node = NULL,
    .mutex = NULL};

/* ===================== 工具函数 ===================== */

static void Init_message_center_mutex(void)
{
    if (g_message_center_mutex == NULL)
    {
        g_message_center_mutex = xSemaphoreCreateMutex();
    }
}

static void Check_name(char *name)
{
    if (name == NULL || strnlen(name, MAX_TOPIC_NAME_LEN + 1) >= MAX_TOPIC_NAME_LEN)
    {
        while (1)
            ;
    }
}

static void Check_len(uint8_t len1, uint8_t len2)
{
    if (len1 != len2)
    {
        while (1)
            ;
    }
}

/* ===================== 接口实现 ===================== */

Publisher_t *Pub_register(char *name, uint8_t data_len)
{
    Check_name(name);
    Init_message_center_mutex();

    if (xSemaphoreTake(g_message_center_mutex, portMAX_DELAY) != pdTRUE)
        return NULL;

    Publisher_t *node = &message_center;
    while (node->next_topic_node)
    {
        node = node->next_topic_node;
        if (strcmp(node->topic_name, name) == 0)
        {
            Check_len(data_len, node->data_len);
            node->pub_registered_flag = 1;
            xSemaphoreGive(g_message_center_mutex);
            return node;
        }
    }

    /* 创建新话题 */
    node->next_topic_node = (Publisher_t *)malloc(sizeof(Publisher_t));
    if (node->next_topic_node == NULL)
    {
        xSemaphoreGive(g_message_center_mutex);
        return NULL;
    }

    memset(node->next_topic_node, 0, sizeof(Publisher_t));
    node->next_topic_node->data_len = data_len;
    strcpy(node->next_topic_node->topic_name, name);
    node->next_topic_node->pub_registered_flag = 1;

    node->next_topic_node->mutex = xSemaphoreCreateMutex();
    if (node->next_topic_node->mutex == NULL)
    {
        free(node->next_topic_node);
        node->next_topic_node = NULL;
        xSemaphoreGive(g_message_center_mutex);
        return NULL;
    }

    xSemaphoreGive(g_message_center_mutex);
    return node->next_topic_node;
}

Subscriber_t *Sub_register(char *name, uint8_t data_len)
{
    Publisher_t *pub = Pub_register(name, data_len);
    if (pub == NULL) return NULL;

    Subscriber_t *ret = (Subscriber_t *)malloc(sizeof(Subscriber_t));
    if (ret == NULL) return NULL;

    memset(ret, 0, sizeof(Subscriber_t));

    ret->mutex = xSemaphoreCreateMutex();
    if (ret->mutex == NULL)
    {
        free(ret);
        return NULL;
    }

    ret->data_len = data_len;
    for (size_t i = 0; i < QUEUE_SIZE; ++i)
    {
        ret->queue[i] = malloc(data_len);
        if (ret->queue[i] == NULL)
        {
            for (size_t j = 0; j < i; ++j) free(ret->queue[j]);
            vSemaphoreDelete(ret->mutex);
            free(ret);
            return NULL;
        }
    }

    /* 挂载到发布者的订阅者链表 */
    if (xSemaphoreTake(pub->mutex, portMAX_DELAY) != pdTRUE)
    {
        for (size_t i = 0; i < QUEUE_SIZE; ++i) free(ret->queue[i]);
        vSemaphoreDelete(ret->mutex);
        free(ret);
        return NULL;
    }

    if (pub->first_subs == NULL)
    {
        pub->first_subs = ret;
        xSemaphoreGive(pub->mutex);
        return ret;
    }

    Subscriber_t *sub = pub->first_subs;
    while (sub->next_subs_queue) sub = sub->next_subs_queue;
    sub->next_subs_queue = ret;

    xSemaphoreGive(pub->mutex);
    return ret;
}

uint8_t Sub_get_message(Subscriber_t *sub, void *data_ptr)
{
    if (sub == NULL || data_ptr == NULL) return 0;

    if (xSemaphoreTake(sub->mutex, portMAX_DELAY) != pdTRUE) return 0;

    if (sub->temp_size == 0)
    {
        xSemaphoreGive(sub->mutex);
        return 0;
    }

    memcpy(data_ptr, sub->queue[sub->front_idx], sub->data_len);
    sub->front_idx = (sub->front_idx + 1) % QUEUE_SIZE;
    sub->temp_size--;

    xSemaphoreGive(sub->mutex);
    return 1;
}

uint8_t Pub_push_message(Publisher_t *pub, void *data_ptr)
{
    if (pub == NULL || data_ptr == NULL) return 0;

    if (xSemaphoreTake(pub->mutex, portMAX_DELAY) != pdTRUE) return 0;

    Subscriber_t *iter = pub->first_subs;
    uint8_t count = 0;

    while (iter)
    {
        if (xSemaphoreTake(iter->mutex, portMAX_DELAY) == pdTRUE)
        {
            if (iter->temp_size == QUEUE_SIZE)
            {
                iter->front_idx = (iter->front_idx + 1) % QUEUE_SIZE;
                iter->temp_size--;
            }
            memcpy(iter->queue[iter->back_idx], data_ptr, pub->data_len);
            iter->back_idx = (iter->back_idx + 1) % QUEUE_SIZE;
            iter->temp_size++;
            count++;

            xSemaphoreGive(iter->mutex);
        }
        iter = iter->next_subs_queue;
    }

    xSemaphoreGive(pub->mutex);
    return count;
}
