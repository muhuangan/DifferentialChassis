/**
  ******************************************************************************
  * @file    mid_chassis_protocol.h
  * @brief   底盘通信帧格式与解析接口。
  * @author  muhuangan
  * @date    2026-10-12
  * @version V1.0.0
  ******************************************************************************
  * @attention
  *
  * Copyright 2026 muhuangan.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  *
  * 本模块是上位机帧协议的唯一替换点，帧头、字段偏移、帧长与载荷布局全部收敛在此处。
  * 模块不依赖 RTOS 与外设，可在宿主机上独立编译与自测。
  *
  * 协议为定长 12 字节，无长度域、无校验和、无使能字段、无急停字段。
  * 载荷按小端序解析，单位脉冲每秒，与驱动器目标速度对象 0x60FF 同量纲。
  *
  ******************************************************************************
  */

#ifndef __MID_CHASSIS_PROTOCOL_H
#define __MID_CHASSIS_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @defgroup MID_CHASSIS_FRAME_Format 底盘通信帧格式
  * @{
  */
#define MID_CHASSIS_FRAME_LENGTH_BYTES        12U    /**< 帧总长度，单位字节。 */
#define MID_CHASSIS_FRAME_HEADER_0            0xAAU  /**< 帧头第 1 字节。 */
#define MID_CHASSIS_FRAME_HEADER_1            0x55U  /**< 帧头第 2 字节。 */
#define MID_CHASSIS_FRAME_TAIL_0              0x0DU  /**< 帧尾第 1 字节。 */
#define MID_CHASSIS_FRAME_TAIL_1              0x0AU  /**< 帧尾第 2 字节。 */
#define MID_CHASSIS_FRAME_OFFSET_HEADER       0U     /**< 帧头偏移。 */
#define MID_CHASSIS_FRAME_OFFSET_LEFT_SPEED   2U     /**< 左轮速度偏移。 */
#define MID_CHASSIS_FRAME_OFFSET_RIGHT_SPEED  6U     /**< 右轮速度偏移。 */
#define MID_CHASSIS_FRAME_OFFSET_TAIL         10U    /**< 帧尾偏移。 */
/**
  * @}
  */

/**
  * @defgroup MID_CHASSIS_FRAME_Second_Byte 底盘通信帧第二字节偏移
  * @{
  */
#define MID_CHASSIS_FRAME_OFFSET_HEADER_1     (MID_CHASSIS_FRAME_OFFSET_HEADER + 1U) /**< 帧头第 2 字节偏移。 */
#define MID_CHASSIS_FRAME_OFFSET_TAIL_1       (MID_CHASSIS_FRAME_OFFSET_TAIL + 1U)   /**< 帧尾第 2 字节偏移。 */
/**
  * @}
  */

/**
  * @brief  一帧合法负载。
  * @note   载荷为小端序 int32，单位脉冲每秒，与 0x60FF 同量纲，不做线速度换算。
  */
typedef struct
{
  int32_t leftSpeed;   /**< 左轮目标转速，单位转每分，由链路层换算成 60FFh 的脉冲每秒。 */
  int32_t rightSpeed;  /**< 右轮目标转速，单位转每分，由链路层换算成 60FFh 的脉冲每秒。 */
} MID_ChassisPayloadTypeDef;

/**
  * @defgroup MID_CHASSIS_PARSE_Result 底盘通信帧解析结果
  * @{
  */
typedef enum
{
  MID_CHASSIS_PARSE_NONE            = 0x00U, /**< 尚未构成完整帧。 */
  MID_CHASSIS_PARSE_VALID           = 0x01U, /**< 解析出合法帧。 */
  MID_CHASSIS_PARSE_HEADER_MISMATCH = 0x02U, /**< 窗口已满但帧头不匹配。 */
  MID_CHASSIS_PARSE_TAIL_MISMATCH   = 0x03U, /**< 帧头匹配但帧尾不匹配。 */
  MID_CHASSIS_PARSE_RANGE_ERROR     = 0x04U  /**< 帧结构合法但速度超出允许范围。 */
} MID_ChassisParseResultTypeDef;
/**
  * @}
  */

void MID_ChassisProtocol_Reset(void);
MID_ChassisParseResultTypeDef MID_ChassisProtocol_ParseByte(uint8_t byte, MID_ChassisPayloadTypeDef *pPayload);

#ifdef __cplusplus
}
#endif

#endif /* __MID_CHASSIS_PROTOCOL_H */
