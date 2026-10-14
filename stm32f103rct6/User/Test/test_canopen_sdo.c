/**
 ******************************************************************************
 * @file test_canopen_sdo.c
 * @brief 验证真实 CANopen 协议栈的应答匹配与超时隔离。
 * @author muhuangan
 * @attention Copyright 2026 muhuangan. MIT License; see project LICENSE.
 ******************************************************************************
 */
#include "mid_canopen.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @defgroup TEST_SDO_Config 测试协议数据
 * @{
 */
#define TEST_NODE 1U
#define TEST_REQUEST_ID 0x601U
#define TEST_RESPONSE_ID 0x581U
#define TEST_OBJECT 0x6041U
#define TEST_ABORT 0x06020000U
#define TEST_READ_COMMAND 0x4BU
#define TEST_READ_VALUE 0x0237U
typedef enum
{
  TEST_REPLY_VALID = 0U,
  TEST_REPLY_WRONG_INDEX,
  TEST_REPLY_WRONG_SUBINDEX,
  TEST_REPLY_WRONG_TYPE,
  TEST_REPLY_SHORT,
  TEST_REPLY_REMOTE,
  TEST_REPLY_EXTENDED,
  TEST_REPLY_ABORT,
  TEST_REPLY_UNSIZED,
  TEST_REPLY_NONE
} TEST_ReplyTypeDef;
/** @} */
static CAN_TypeDef         s_instance;
static CAN_HandleTypeDef   s_hcan = {&s_instance};
static CAN_RxHeaderTypeDef s_header;
static uint8_t             s_data[MID_CANOPEN_FRAME_DATA_SIZE_BYTES];
static TEST_ReplyTypeDef   s_reply;
static uint32_t            s_tick;
static uint32_t            s_abortCount;
static uint32_t            s_checks;
static uint8_t             s_lastRequest[MID_CANOPEN_FRAME_DATA_SIZE_BYTES];

/** @brief 失败时输出断言行号并退出。 */
static void TEST_Check(bool condition, int line)
{
  s_checks++;
  if (!condition)
  {
    fprintf(stderr, "FAIL at line %d\n", line);
    exit(EXIT_FAILURE);
  }
}
#define CHECK(condition) TEST_Check((condition), __LINE__)

/** @brief 单调时钟推进真实协议栈的超时分支。 */
uint32_t HAL_GetTick(void)
{
  return s_tick++;
}
/** @brief 模拟滤波器配置成功。 */
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *pFilter)
{
  (void) hcan;
  (void) pFilter;
  return HAL_OK;
}
/** @brief 模拟启动成功。 */
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan)
{
  (void) hcan;
  return HAL_OK;
}
/** @brief 模拟停止成功。 */
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan)
{
  (void) hcan;
  return HAL_OK;
}
/** @brief 模拟中断使能成功。 */
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t flags)
{
  (void) hcan;
  (void) flags;
  return HAL_OK;
}
/** @brief 保持一个可用发送邮箱。 */
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan)
{
  (void) hcan;
  return 1U;
}
/** @brief 检查超时是否取消挂起的发送。 */
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, uint32_t mailbox)
{
  (void) hcan;
  CHECK(mailbox == 1U);
  s_abortCount++;
  return HAL_OK;
}
/** @brief 向真实中断解析逻辑返回模拟帧。 */
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t fifo, CAN_RxHeaderTypeDef *pHeader,
                                       uint8_t *pData)
{
  (void) hcan;
  (void) fifo;
  *pHeader = s_header;
  (void) memcpy(pData, s_data, sizeof(s_data));
  return HAL_OK;
}
/**
 * @brief 按请求生成应答并注入错误后触发真实接收中断。
 * @note 错误应答后不补发正确帧，必须超时而不能被误判成功。
 */
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t *pData,
                                       uint32_t *pMailbox)
{
  CHECK(pHeader->StdId == TEST_REQUEST_ID);
  CHECK(pHeader->IDE == CAN_ID_STD && pHeader->RTR == CAN_RTR_DATA);
  CHECK(pHeader->DLC == MID_CANOPEN_FRAME_DATA_SIZE_BYTES);
  (void) memcpy(s_lastRequest, pData, sizeof(s_lastRequest));
  CHECK(pData[0] == 0x40U || pData[0] == 0x2FU || pData[0] == 0x2BU || pData[0] == 0x27U || pData[0] == 0x23U);
  *pMailbox = 1U;
  (void) memset(&s_header, 0, sizeof(s_header));
  s_header.StdId = TEST_RESPONSE_ID;
  s_header.DLC = sizeof(s_data);
  (void) memcpy(s_data, pData, sizeof(s_data));
  s_data[0] = (pData[0] == MID_CANOPEN_SDO_CCS_UPLOAD_REQUEST) ? TEST_READ_COMMAND : MID_CANOPEN_SDO_SCS_DOWNLOAD_ACK;
  s_data[4] = (uint8_t) TEST_READ_VALUE;
  s_data[5] = (uint8_t) (TEST_READ_VALUE >> 8U);
  s_data[6] = 0U;
  s_data[7] = 0U;
  switch (s_reply)
  {
    case TEST_REPLY_WRONG_INDEX:
      s_data[1]++;
      break;
    case TEST_REPLY_WRONG_SUBINDEX:
      s_data[3]++;
      break;
    case TEST_REPLY_WRONG_TYPE:
      s_data[0] = MID_CANOPEN_SDO_SCS_DOWNLOAD_ACK;
      break;
    case TEST_REPLY_SHORT:
      s_header.DLC--;
      break;
    case TEST_REPLY_REMOTE:
      s_header.RTR = 1U;
      break;
    case TEST_REPLY_EXTENDED:
      s_header.IDE = 1U;
      break;
    case TEST_REPLY_ABORT:
      s_data[0] = MID_CANOPEN_SDO_SCS_ABORT;
      s_data[4] = (uint8_t) TEST_ABORT;
      s_data[5] = (uint8_t) (TEST_ABORT >> 8U);
      s_data[6] = (uint8_t) (TEST_ABORT >> 16U);
      s_data[7] = (uint8_t) (TEST_ABORT >> 24U);
      break;
    case TEST_REPLY_UNSIZED:
      s_data[0] = MID_CANOPEN_SDO_SCS_UPLOAD_RESPONSE;
      break;
    case TEST_REPLY_NONE:
      return HAL_OK;
    default:
      break;
  }
  HAL_CAN_RxFifo0MsgPendingCallback(hcan);
  return HAL_OK;
}
/**
 * @brief 验证合法、错配、中止、迟到和无应答事务。
 * @note 失败读保留原值，迟到帧不能重新打开已结束事务。
 */
int main(void)
{
  /* 直接对照手册原始帧，避免用组帧函数自身的宏生成期望值。 */
  const uint8_t           readStatus[] = {0x40U, 0x41U, 0x60U, 0U, 0U, 0U, 0U, 0U};
  const uint8_t           writeMode[] = {0x2FU, 0x60U, 0x60U, 0U, 1U, 0U, 0U, 0U};
  const uint8_t           writeSwitch[] = {0x2BU, 0x40U, 0x60U, 0U, 7U, 0U, 0U, 0U};
  const uint8_t           writeArm[] = {0x2BU, 0x40U, 0x60U, 0U, 0x4FU, 0U, 0U, 0U};
  const uint8_t           writeTrigger[] = {0x2BU, 0x40U, 0x60U, 0U, 0x5FU, 0U, 0U, 0U};
  const uint8_t           writeSpeed[] = {0x23U, 0x81U, 0x60U, 0U, 0xF4U, 1U, 0U, 0U};
  const uint8_t           writeRamp[] = {0x23U, 0x83U, 0x60U, 0U, 0xE8U, 3U, 0U, 0U};
  const uint8_t           writeNegativePosition[] = {0x23U, 0x7AU, 0x60U, 0U, 0x38U, 0xFFU, 0xFFU, 0xFFU};
  const uint8_t           threeBytes[] = {1U, 2U, 3U};
  const uint8_t           writeThreeBytes[] = {0x27U, 0x81U, 0x60U, 0U, 1U, 2U, 3U, 0U};
  uint16_t                value = 0U;
  MID_CANopenNodeTypeDef *pNode;
  CHECK(MID_CANopen_Init(&s_hcan) == MID_CANOPEN_OK);
  CHECK(MID_CANopen_AddNode(TEST_NODE) == MID_CANOPEN_OK);
  pNode = MID_CANopen_GetNode(TEST_NODE);
  CHECK(pNode != NULL);
  CHECK(MID_CANopen_Sdo_ReadU16(TEST_NODE, TEST_OBJECT, 0U, &value) == MID_CANOPEN_OK);
  CHECK(value == TEST_READ_VALUE);
  CHECK(memcmp(s_lastRequest, readStatus, sizeof(readStatus)) == 0);
  CHECK(MID_CANopen_Sdo_WriteU8(TEST_NODE, 0x6060U, 0U, 1U) == MID_CANOPEN_OK);
  CHECK(memcmp(s_lastRequest, writeMode, sizeof(writeMode)) == 0);
  CHECK(MID_CANopen_Sdo_WriteU16(TEST_NODE, 0x6040U, 0U, 7U) == MID_CANOPEN_OK);
  CHECK(memcmp(s_lastRequest, writeSwitch, sizeof(writeSwitch)) == 0);
  CHECK(MID_CANopen_Sdo_WriteU16(TEST_NODE, 0x6040U, 0U, 0x4FU) == MID_CANOPEN_OK);
  CHECK(memcmp(s_lastRequest, writeArm, sizeof(writeArm)) == 0);
  CHECK(MID_CANopen_Sdo_WriteU16(TEST_NODE, 0x6040U, 0U, 0x5FU) == MID_CANOPEN_OK);
  CHECK(memcmp(s_lastRequest, writeTrigger, sizeof(writeTrigger)) == 0);
  CHECK(MID_CANopen_Sdo_WriteU32(TEST_NODE, 0x6081U, 0U, 500U) == MID_CANOPEN_OK);
  CHECK(memcmp(s_lastRequest, writeSpeed, sizeof(writeSpeed)) == 0);
  CHECK(MID_CANopen_Sdo_WriteU32(TEST_NODE, 0x6083U, 0U, 1000U) == MID_CANOPEN_OK);
  CHECK(memcmp(s_lastRequest, writeRamp, sizeof(writeRamp)) == 0);
  CHECK(MID_CANopen_Sdo_WriteI32(TEST_NODE, 0x607AU, 0U, -200) == MID_CANOPEN_OK);
  CHECK(memcmp(s_lastRequest, writeNegativePosition, sizeof(writeNegativePosition)) == 0);
  CHECK(MID_CANopen_Sdo_Write(TEST_NODE, 0x6081U, 0U, threeBytes, sizeof(threeBytes)) == MID_CANOPEN_OK);
  CHECK(memcmp(s_lastRequest, writeThreeBytes, sizeof(writeThreeBytes)) == 0);
  for (s_reply = TEST_REPLY_WRONG_INDEX; s_reply <= TEST_REPLY_EXTENDED; s_reply++)
  {
    value = 0U;
    CHECK(MID_CANopen_Sdo_ReadU16(TEST_NODE, TEST_OBJECT, 0U, &value) == MID_CANOPEN_TIMEOUT);
    CHECK(value == 0U);
    CHECK(!pNode->isSdoWaiting);
  }
  CHECK(s_abortCount == 6U);
  s_reply = TEST_REPLY_ABORT;
  CHECK(MID_CANopen_Sdo_ReadU16(TEST_NODE, TEST_OBJECT, 0U, &value) == MID_CANOPEN_ABORTED);
  CHECK(pNode->lastSdoAbortCode == TEST_ABORT);
  s_reply = TEST_REPLY_VALID;
  CHECK(MID_CANopen_Sdo_ReadU16(TEST_NODE, TEST_OBJECT, 0U, &value) == MID_CANOPEN_OK);
  CHECK(pNode->lastSdoAbortCode == 0U);
  s_reply = TEST_REPLY_UNSIZED;
  CHECK(MID_CANopen_Sdo_ReadU16(TEST_NODE, TEST_OBJECT, 0U, &value) == MID_CANOPEN_ERROR);
  s_reply = TEST_REPLY_NONE;
  CHECK(MID_CANopen_Sdo_ReadU16(TEST_NODE, TEST_OBJECT, 0U, &value) == MID_CANOPEN_TIMEOUT);
  s_data[0] = TEST_READ_COMMAND;
  HAL_CAN_RxFifo0MsgPendingCallback(&s_hcan);
  CHECK(!pNode->hasSdoResponse);
  CHECK(!pNode->isSdoWaiting);
  printf("PASS: %lu CANopen SDO checks.\n", (unsigned long) s_checks);
  return EXIT_SUCCESS;
}
