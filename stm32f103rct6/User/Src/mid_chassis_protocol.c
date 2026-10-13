/**
  ******************************************************************************
  * @file    mid_chassis_protocol.c
  * @brief   底盘通信帧解析状态机实现。
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
  ******************************************************************************
  */

#include "mid_chassis_protocol.h"

#include <stdbool.h>
#include <stddef.h>

#include "chassis_link_config.h"

/**
  * @defgroup MID_CHASSIS_PARSE_Window 解析窗口尺寸
  * @{
  */
#define MID_CHASSIS_PARSE_WINDOW_SHIFT  (MID_CHASSIS_FRAME_LENGTH_BYTES - 1U) /**< 窗口滑动一个字节后保留的长度。 */
/**
  * @}
  */

static uint8_t s_window[MID_CHASSIS_FRAME_LENGTH_BYTES];   /**< 定长滑动窗口，填满一窗即校验帧头与帧尾。 */
static uint8_t s_windowLength = 0U;                        /**< 当前窗口内已填充的字节数。 */

static void MID_ChassisProtocol_Slide(void);
static int32_t MID_ChassisProtocol_ToInt32(const uint8_t *pBytes);
static bool MID_ChassisProtocol_IsSpeedInRange(int32_t speed);

/**
  * @brief  滑动窗口左移一个字节以重新同步。
  * @param  None
  * @retval None
  * @note   丢弃窗口中最旧的一个字节并让其余字节前移，窗口长度回退到 11，
  *         使下一个新字节仍能凑满 12 字节窗口再次参与校验。
  */
static void MID_ChassisProtocol_Slide(void)
{
  uint8_t index = 0U;

  for (index = 0U; index < MID_CHASSIS_PARSE_WINDOW_SHIFT; index++)
  {
    s_window[index] = s_window[index + 1U];
  }

  s_windowLength = MID_CHASSIS_PARSE_WINDOW_SHIFT;
}

/**
  * @brief  按小端序把 4 个字节拼成有符号整数。
  * @param  pBytes  指向 4 字节数据首字节的指针，不可为 NULL。
  * @retval 拼出的 32 位有符号数值。
  * @note   首字节为最低位，与驱动器 0x60FF 等对象的小端表示一致。
  */
static int32_t MID_ChassisProtocol_ToInt32(const uint8_t *pBytes)
{
  uint32_t value = 0U;

  value = ((uint32_t)pBytes[0]) |
          ((uint32_t)pBytes[1] << 8U) |
          ((uint32_t)pBytes[2] << 16U) |
          ((uint32_t)pBytes[3] << 24U);

  return (int32_t)value;
}

/**
  * @brief  校验速度是否在允许范围内。
  * @param  speed  待校验速度，单位脉冲每秒。
  * @retval true   绝对值不超过 CHASSIS_LINK_SPEED_LIMIT_PPS。
  * @retval false  超出限幅，对应帧被判为非法。
  * @note   限幅值取自 CHASSIS_LINK_SPEED_LIMIT_PPS，任一轮超限即整帧丢弃。
  */
static bool MID_ChassisProtocol_IsSpeedInRange(int32_t speed)
{
  return ((speed <= CHASSIS_LINK_SPEED_LIMIT_PPS) && (speed >= -CHASSIS_LINK_SPEED_LIMIT_PPS));
}

/**
  * @brief  清空解析窗口，丢弃半帧数据重新开始同步。
  * @param  None
  * @retval None
  * @note   把窗口内全部字节清零并将窗口长度归零，下一个字节从窗口起点重新累积。
  */
void MID_ChassisProtocol_Reset(void)
{
  uint8_t index = 0U;

  for (index = 0U; index < MID_CHASSIS_FRAME_LENGTH_BYTES; index++)
  {
    s_window[index] = 0U;
  }

  s_windowLength = 0U;
}

/**
  * @brief  向解析器投喂一个字节，窗口凑满后校验并取出负载。
  * @param  byte      待解析字节。
  * @param  pPayload  解析出合法帧时写入负载的缓冲区指针，可为 NULL 表示只判断帧的合法性。
  * @retval MID_CHASSIS_PARSE_NONE            窗口未满，尚未构成完整帧。
  * @retval MID_CHASSIS_PARSE_VALID           帧头帧尾与速度均合法，负载已写出。
  * @retval MID_CHASSIS_PARSE_HEADER_MISMATCH 帧头不匹配，窗口已滑动一个字节。
  * @retval MID_CHASSIS_PARSE_TAIL_MISMATCH   帧尾不匹配，窗口已滑动一个字节。
  * @retval MID_CHASSIS_PARSE_RANGE_ERROR     速度超出限幅，窗口已滑动一个字节。
  * @note   处理顺序：把字节压入窗口；窗口未满直接返回；校验两字节帧头；校验两字节帧尾；
  *         按小端序取出左右轮速度并做限幅校验；全部通过才清空窗口并写出负载。
  *         任一步失败都先滑动一个字节再返回，因此不会因个别错误字节卡死；
  *         帧尾只按固定偏移校验，不做字节流扫描，避免与载荷中的 0x0D 0x0A 产生歧义。
  */
MID_ChassisParseResultTypeDef MID_ChassisProtocol_ParseByte(uint8_t byte, MID_ChassisPayloadTypeDef *pPayload)
{
  int32_t leftSpeed = 0;
  int32_t rightSpeed = 0;

  s_window[s_windowLength] = byte;
  s_windowLength++;

  if (s_windowLength < MID_CHASSIS_FRAME_LENGTH_BYTES)
  {
    return MID_CHASSIS_PARSE_NONE;
  }

  if ((s_window[MID_CHASSIS_FRAME_OFFSET_HEADER] != MID_CHASSIS_FRAME_HEADER_0) ||
      (s_window[MID_CHASSIS_FRAME_OFFSET_HEADER_1] != MID_CHASSIS_FRAME_HEADER_1))
  {
    MID_ChassisProtocol_Slide();

    return MID_CHASSIS_PARSE_HEADER_MISMATCH;
  }

  if ((s_window[MID_CHASSIS_FRAME_OFFSET_TAIL] != MID_CHASSIS_FRAME_TAIL_0) ||
      (s_window[MID_CHASSIS_FRAME_OFFSET_TAIL_1] != MID_CHASSIS_FRAME_TAIL_1))
  {
    MID_ChassisProtocol_Slide();

    return MID_CHASSIS_PARSE_TAIL_MISMATCH;
  }

  leftSpeed = MID_ChassisProtocol_ToInt32(&s_window[MID_CHASSIS_FRAME_OFFSET_LEFT_SPEED]);
  rightSpeed = MID_ChassisProtocol_ToInt32(&s_window[MID_CHASSIS_FRAME_OFFSET_RIGHT_SPEED]);

  if ((MID_ChassisProtocol_IsSpeedInRange(leftSpeed) == false) ||
      (MID_ChassisProtocol_IsSpeedInRange(rightSpeed) == false))
  {
    MID_ChassisProtocol_Slide();

    return MID_CHASSIS_PARSE_RANGE_ERROR;
  }

  s_windowLength = 0U;

  if (pPayload != NULL)
  {
    pPayload->leftSpeed = leftSpeed;
    pPayload->rightSpeed = rightSpeed;
  }

  return MID_CHASSIS_PARSE_VALID;
}
