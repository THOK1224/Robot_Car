/**
 * @file    message_center.h
 * @brief   消息中心 (发布-订阅模式)
 * @author  SYSU电控组
 * @note    移植自 SYSU_Hero
 */
#ifndef PUBSUB_H
#define PUBSUB_H

#include "stdint.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define MAX_TOPIC_NAME_LEN 32
#define MAX_TOPIC_COUNT 12
#define QUEUE_SIZE 1

/**
 * @brief 订阅者实例
 */
typedef struct mqt
{
    void *queue[QUEUE_SIZE];
    uint8_t data_len;
    uint8_t front_idx;
    uint8_t back_idx;
    uint8_t temp_size;

    SemaphoreHandle_t mutex;
    struct mqt *next_subs_queue;
} Subscriber_t;

/**
 * @brief 发布者实例
 */
typedef struct ent
{
    char topic_name[MAX_TOPIC_NAME_LEN + 1];
    uint8_t data_len;
    Subscriber_t *first_subs;
    struct ent *next_topic_node;
    uint8_t pub_registered_flag;

    SemaphoreHandle_t mutex;
} Publisher_t;

/**
 * @brief 订阅话题
 * @return 订阅者实例指针
 */
Subscriber_t *Sub_register(char *name, uint8_t data_len);

/**
 * @brief 注册为发布者
 * @return 发布者实例指针
 */
Publisher_t *Pub_register(char *name, uint8_t data_len);

/**
 * @brief 获取订阅消息
 * @return 0=无新消息, 1=获取成功
 */
uint8_t Sub_get_message(Subscriber_t *sub, void *data_ptr);

/**
 * @brief 推送消息给所有订阅者
 * @return 成功推送的订阅者数量
 */
uint8_t Pub_push_message(Publisher_t *pub, void *data_ptr);

#endif // !PUBSUB_H
