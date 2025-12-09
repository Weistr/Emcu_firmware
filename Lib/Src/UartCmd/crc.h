#ifndef _CRC_H
#define _CRC_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief CRC16-CCITT 多项式 (0x1021)
 * 初始值: 0xFFFF
 */
#define CRC16_CCITT_POLY 0x1021
#define CRC16_CCITT_INIT 0xFFFF

/**
 * @brief 计算数据的 CRC16-CCITT 校验值
 * 
 * @param data 数据指针
 * @param length 数据长度
 * @return uint16_t CRC16 校验值
 */
uint16_t crc16_ccitt(const uint8_t *data, size_t length);

/**
 * @brief 计算数据的 CRC16-CCITT 校验值（带初始值）
 * 
 * @param crc 初始 CRC 值
 * @param data 数据指针
 * @param length 数据长度
 * @return uint16_t CRC16 校验值
 */
uint16_t crc16_ccitt_update(uint16_t crc, const uint8_t *data, size_t length);

/**
 * @brief 验证数据的 CRC16 校验
 * 
 * @param data 数据指针（包含 CRC）
 * @param length 数据总长度（包括2字节CRC）
 * @return uint8_t 0: 校验成功, 非0: 校验失败
 */
uint8_t crc16_verify(const uint8_t *data, size_t length);

#endif // _CRC_H
