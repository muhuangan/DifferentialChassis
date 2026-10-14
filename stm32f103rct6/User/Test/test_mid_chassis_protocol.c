/**
  ******************************************************************************
  * @file    test_mid_chassis_protocol.c
  * @brief   底盘通信帧解析状态机的宿主机自测用例。
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
  * 本文件只用于宿主机自测，不参与固件构建。
  * 编译与运行示例：
  *   gcc -std=c99 -Wall -Wextra -I../Inc -I../Config -o /tmp/proto_test test_mid_chassis_protocol.c ../Src/mid_chassis_protocol.c
  *   /tmp/proto_test
  *
  ******************************************************************************
  */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "chassis_link_config.h"
#include "mid_chassis_protocol.h"

/**
  * @brief  单次投喂过程的解析结果计数。
  */
typedef struct
{
  uint32_t noneCount;           /**< 尚未构成完整帧的次数。 */
  uint32_t validCount;          /**< 合法帧次数。 */
  uint32_t headerMismatchCount; /**< 帧头失配次数。 */
  uint32_t tailMismatchCount;   /**< 帧尾失配次数。 */
  uint32_t rangeErrorCount;     /**< 范围校验失败次数。 */
  MID_ChassisPayloadTypeDef lastPayload; /**< 最近一次合法帧的载荷。 */
} TestCountersTypeDef;

static uint32_t s_checkCount = 0U;
static uint32_t s_failureCount = 0U;

static void Test_Check(bool condition, const char *pName);
static void Test_BuildFrame(uint8_t *pFrame, int32_t leftSpeed, int32_t rightSpeed);
static void Test_Feed(const uint8_t *pBytes, uint16_t length, TestCountersTypeDef *pCounters);
static void Test_ResetCounters(TestCountersTypeDef *pCounters);
static void Test_ValidFrame(void);
static void Test_TwoFramesInOneChunk(void);
static void Test_SplitDelivery(void);
static void Test_PayloadContainsTailBytes(void);
static void Test_LeadingGarbageResync(void);
static void Test_TailMismatchThenResync(void);
static void Test_RangeLimits(void);
static void Test_ResetMidStream(void);

static void Test_Check(bool condition, const char *pName)
{
  s_checkCount++;

  if (condition == false)
  {
    s_failureCount++;
    (void)printf("FAIL: %s\n", pName);
  }
}

static void Test_BuildFrame(uint8_t *pFrame, int32_t leftSpeed, int32_t rightSpeed)
{
  uint32_t left = (uint32_t)leftSpeed;
  uint32_t right = (uint32_t)rightSpeed;

  pFrame[MID_CHASSIS_FRAME_OFFSET_HEADER] = MID_CHASSIS_FRAME_HEADER_0;
  pFrame[MID_CHASSIS_FRAME_OFFSET_HEADER_1] = MID_CHASSIS_FRAME_HEADER_1;

  pFrame[MID_CHASSIS_FRAME_OFFSET_LEFT_SPEED] = (uint8_t)(left & 0xFFU);
  pFrame[MID_CHASSIS_FRAME_OFFSET_LEFT_SPEED + 1U] = (uint8_t)((left >> 8U) & 0xFFU);
  pFrame[MID_CHASSIS_FRAME_OFFSET_LEFT_SPEED + 2U] = (uint8_t)((left >> 16U) & 0xFFU);
  pFrame[MID_CHASSIS_FRAME_OFFSET_LEFT_SPEED + 3U] = (uint8_t)((left >> 24U) & 0xFFU);

  pFrame[MID_CHASSIS_FRAME_OFFSET_RIGHT_SPEED] = (uint8_t)(right & 0xFFU);
  pFrame[MID_CHASSIS_FRAME_OFFSET_RIGHT_SPEED + 1U] = (uint8_t)((right >> 8U) & 0xFFU);
  pFrame[MID_CHASSIS_FRAME_OFFSET_RIGHT_SPEED + 2U] = (uint8_t)((right >> 16U) & 0xFFU);
  pFrame[MID_CHASSIS_FRAME_OFFSET_RIGHT_SPEED + 3U] = (uint8_t)((right >> 24U) & 0xFFU);

  pFrame[MID_CHASSIS_FRAME_OFFSET_TAIL] = MID_CHASSIS_FRAME_TAIL_0;
  pFrame[MID_CHASSIS_FRAME_OFFSET_TAIL_1] = MID_CHASSIS_FRAME_TAIL_1;
}

static void Test_ResetCounters(TestCountersTypeDef *pCounters)
{
  pCounters->noneCount = 0U;
  pCounters->validCount = 0U;
  pCounters->headerMismatchCount = 0U;
  pCounters->tailMismatchCount = 0U;
  pCounters->rangeErrorCount = 0U;
  pCounters->lastPayload.leftSpeed = 0;
  pCounters->lastPayload.rightSpeed = 0;
}

static void Test_Feed(const uint8_t *pBytes, uint16_t length, TestCountersTypeDef *pCounters)
{
  MID_ChassisPayloadTypeDef payload;
  MID_ChassisParseResultTypeDef result = MID_CHASSIS_PARSE_NONE;
  uint16_t index = 0U;

  for (index = 0U; index < length; index++)
  {
    payload.leftSpeed = 0;
    payload.rightSpeed = 0;
    result = MID_ChassisProtocol_ParseByte(pBytes[index], &payload);

    switch (result)
    {
      case MID_CHASSIS_PARSE_NONE:
        pCounters->noneCount++;
        break;
      case MID_CHASSIS_PARSE_VALID:
        pCounters->validCount++;
        pCounters->lastPayload = payload;
        break;
      case MID_CHASSIS_PARSE_HEADER_MISMATCH:
        pCounters->headerMismatchCount++;
        break;
      case MID_CHASSIS_PARSE_TAIL_MISMATCH:
        pCounters->tailMismatchCount++;
        break;
      case MID_CHASSIS_PARSE_RANGE_ERROR:
        pCounters->rangeErrorCount++;
        break;
      default:
        break;
    }
  }
}

static void Test_ValidFrame(void)
{
  uint8_t frame[MID_CHASSIS_FRAME_LENGTH_BYTES];
  TestCountersTypeDef counters;

  Test_ResetCounters(&counters);
  MID_ChassisProtocol_Reset();
  Test_BuildFrame(frame, 40, -60);
  Test_Feed(frame, MID_CHASSIS_FRAME_LENGTH_BYTES, &counters);

  Test_Check(counters.validCount == 1U, "valid frame: one valid result");
  Test_Check(counters.noneCount == (MID_CHASSIS_FRAME_LENGTH_BYTES - 1U), "valid frame: no premature result");
  Test_Check(counters.headerMismatchCount == 0U, "valid frame: no header mismatch");
  Test_Check(counters.tailMismatchCount == 0U, "valid frame: no tail mismatch");
  Test_Check(counters.lastPayload.leftSpeed == 40, "valid frame: left payload");
  Test_Check(counters.lastPayload.rightSpeed == -60, "valid frame: right payload");
}

static void Test_TwoFramesInOneChunk(void)
{
  uint8_t bytes[2U * MID_CHASSIS_FRAME_LENGTH_BYTES];
  TestCountersTypeDef counters;

  Test_ResetCounters(&counters);
  MID_ChassisProtocol_Reset();
  Test_BuildFrame(&bytes[0], 40, 50);
  Test_BuildFrame(&bytes[MID_CHASSIS_FRAME_LENGTH_BYTES], -30, -40);
  Test_Feed(bytes, (uint16_t)sizeof(bytes), &counters);

  Test_Check(counters.validCount == 2U, "back to back: two valid results");
  Test_Check(counters.lastPayload.leftSpeed == -30, "back to back: second left payload");
  Test_Check(counters.lastPayload.rightSpeed == -40, "back to back: second right payload");
}

static void Test_SplitDelivery(void)
{
  uint8_t frame[MID_CHASSIS_FRAME_LENGTH_BYTES];
  TestCountersTypeDef counters;

  Test_ResetCounters(&counters);
  MID_ChassisProtocol_Reset();
  Test_BuildFrame(frame, 7, -9);
  Test_Feed(&frame[0], 5U, &counters);
  Test_Feed(&frame[5], 4U, &counters);
  Test_Feed(&frame[9], 3U, &counters);

  Test_Check(counters.validCount == 1U, "split delivery: one valid result");
  Test_Check(counters.lastPayload.leftSpeed == 7, "split delivery: left payload");
  Test_Check(counters.lastPayload.rightSpeed == -9, "split delivery: right payload");
}

static void Test_PayloadContainsTailBytes(void)
{
  uint8_t frame[MID_CHASSIS_FRAME_LENGTH_BYTES];
  TestCountersTypeDef counters;

  Test_ResetCounters(&counters);
  MID_ChassisProtocol_Reset();

  /* 速度值取 10 与 13，小端末字节分别为 0A 与 0D，与帧尾序列同字节，用于验证帧尾只按固定偏移校验。 */
  Test_BuildFrame(frame, 10, 13);
  Test_Feed(frame, MID_CHASSIS_FRAME_LENGTH_BYTES, &counters);

  Test_Check(counters.validCount == 1U, "tail-like payload: one valid result");
  Test_Check(counters.lastPayload.leftSpeed == 10, "tail-like payload: left payload");
  Test_Check(counters.lastPayload.rightSpeed == 13, "tail-like payload: right payload");
}

static void Test_LeadingGarbageResync(void)
{
  uint8_t bytes[6U + MID_CHASSIS_FRAME_LENGTH_BYTES];
  TestCountersTypeDef counters;
  uint16_t index = 0U;

  for (index = 0U; index < 6U; index++)
  {
    bytes[index] = (uint8_t)(0xA0U + index);
  }

  Test_ResetCounters(&counters);
  MID_ChassisProtocol_Reset();
  Test_BuildFrame(&bytes[6], 55, 66);
  Test_Feed(bytes, (uint16_t)sizeof(bytes), &counters);

  Test_Check(counters.validCount == 1U, "leading garbage: one valid result");
  Test_Check(counters.headerMismatchCount > 0U, "leading garbage: header mismatch reported");
  Test_Check(counters.lastPayload.leftSpeed == 55, "leading garbage: left payload");
  Test_Check(counters.lastPayload.rightSpeed == 66, "leading garbage: right payload");
}

static void Test_TailMismatchThenResync(void)
{
  uint8_t bytes[2U * MID_CHASSIS_FRAME_LENGTH_BYTES];
  TestCountersTypeDef counters;

  Test_ResetCounters(&counters);
  MID_ChassisProtocol_Reset();
  Test_BuildFrame(&bytes[0], 11, 22);
  bytes[MID_CHASSIS_FRAME_OFFSET_TAIL] = 0x00U; /* 破坏首帧帧尾。 */
  Test_BuildFrame(&bytes[MID_CHASSIS_FRAME_LENGTH_BYTES], 33, 44);
  Test_Feed(bytes, (uint16_t)sizeof(bytes), &counters);

  Test_Check(counters.tailMismatchCount == 1U, "broken tail: exactly one tail mismatch");
  Test_Check(counters.validCount == 1U, "broken tail: following frame recovered");
  Test_Check(counters.lastPayload.leftSpeed == 33, "broken tail: following left payload");
  Test_Check(counters.lastPayload.rightSpeed == 44, "broken tail: following right payload");
}

static void Test_RangeLimits(void)
{
  uint8_t frame[MID_CHASSIS_FRAME_LENGTH_BYTES];
  TestCountersTypeDef counters;

  Test_ResetCounters(&counters);
  MID_ChassisProtocol_Reset();
  Test_BuildFrame(frame, CHASSIS_LINK_SPEED_LIMIT_RPM, -CHASSIS_LINK_SPEED_LIMIT_RPM);
  Test_Feed(frame, MID_CHASSIS_FRAME_LENGTH_BYTES, &counters);
  Test_Check(counters.validCount == 1U, "range limit: boundary values accepted");
  Test_Check(counters.lastPayload.leftSpeed == CHASSIS_LINK_SPEED_LIMIT_RPM, "range limit: left boundary");
  Test_Check(counters.lastPayload.rightSpeed == -CHASSIS_LINK_SPEED_LIMIT_RPM, "range limit: right boundary");

  Test_ResetCounters(&counters);
  MID_ChassisProtocol_Reset();
  Test_BuildFrame(frame, CHASSIS_LINK_SPEED_LIMIT_RPM + 1, 0);
  Test_Feed(frame, MID_CHASSIS_FRAME_LENGTH_BYTES, &counters);
  Test_Check(counters.rangeErrorCount == 1U, "range limit: over limit rejected");
  Test_Check(counters.validCount == 0U, "range limit: over limit yields no valid frame");

  Test_ResetCounters(&counters);
  MID_ChassisProtocol_Reset();
  Test_BuildFrame(frame, 0, INT32_MIN);
  Test_Feed(frame, MID_CHASSIS_FRAME_LENGTH_BYTES, &counters);
  Test_Check(counters.rangeErrorCount == 1U, "range limit: minimum int32 rejected");
  Test_Check(counters.validCount == 0U, "range limit: minimum int32 yields no valid frame");
}

static void Test_ResetMidStream(void)
{
  uint8_t frame[MID_CHASSIS_FRAME_LENGTH_BYTES];
  TestCountersTypeDef counters;

  Test_ResetCounters(&counters);
  MID_ChassisProtocol_Reset();
  Test_BuildFrame(frame, 1, 2);
  Test_Feed(&frame[0], 6U, &counters);
  MID_ChassisProtocol_Reset();
  Test_Feed(frame, MID_CHASSIS_FRAME_LENGTH_BYTES, &counters);

  Test_Check(counters.validCount == 1U, "reset mid stream: one valid result after reset");
  Test_Check(counters.lastPayload.leftSpeed == 1, "reset mid stream: left payload");
  Test_Check(counters.lastPayload.rightSpeed == 2, "reset mid stream: right payload");
}

int main(void)
{
  Test_ValidFrame();
  Test_TwoFramesInOneChunk();
  Test_SplitDelivery();
  Test_PayloadContainsTailBytes();
  Test_LeadingGarbageResync();
  Test_TailMismatchThenResync();
  Test_RangeLimits();
  Test_ResetMidStream();

  (void)printf("checks=%u failures=%u\n", (unsigned int)s_checkCount, (unsigned int)s_failureCount);

  return ((s_failureCount == 0U) ? 0 : 1);
}
