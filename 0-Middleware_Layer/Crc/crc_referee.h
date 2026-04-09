/**
 * @file    crc_referee.h
 * @brief   CRC8/CRC16 校验模块
 * @author  SYSU电控组
 * @note    移植自 SYSU_Hero
 */
#ifndef _CRC_REFEREE_H
#define _CRC_REFEREE_H

#include "main.h"

uint8_t  crc_8(const uint8_t *input_str, uint16_t num_bytes);
uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes);
uint16_t get_crc16_check_sum(const uint8_t *data, uint32_t len);

uint8_t  Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
void     Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
uint16_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void     Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif
