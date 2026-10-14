/**
 ******************************************************************************
 * @file    test_motor_startup.c
 * @brief   通过模拟对象字典验证启动记录、假应答、读回超时和停止失败。
 * @author  muhuangan
 * @date    2026-10-13
 * @version V1.0.0
 ******************************************************************************
 * @attention
 * Copyright 2026 muhuangan.
 * This software is licensed under the MIT License in the project LICENSE file.
 ******************************************************************************
 */
#include "app_motor_startup.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "chassis_link_config.h"

/** @defgroup TEST_MOTOR_Config 模拟对象字典与时钟配置
 * @{
 */
#define TEST_NODE_COUNT 2U
#define TEST_TICK_MS 10U
#define TEST_HEARTBEAT_MS 1000U
#define TEST_LOOPS 2000U
#define TEST_READY_STATUS 0x0231U
#define TEST_SWITCHED_STATUS 0x0233U
#define TEST_ENABLED_STATUS 0x0237U
#define TEST_DISABLED_STATUS 0x0250U
#define TEST_FAULT_STATUS 0x0218U
#define TEST_ERROR_CODE 0x3122U
#define TEST_SPEED_PPS 5000
/** @} */

typedef struct
{
  MID_CANopenNodeTypeDef node;
  uint16_t               statusWord;
  uint16_t               errorCode;
  int8_t                 mode;
  uint32_t               acceleration;
  uint32_t               deceleration;
  int32_t                targetVelocity;
  int32_t                actualVelocity;
  int32_t                targetPosition;
  int32_t                actualPosition;
  uint32_t               profileSpeed;
  uint32_t               triggerCount;
  uint32_t               rampWriteCount;
  uint32_t               velocityWriteCount;
  bool                   freezePosition;
  bool                   loseTriggerAck;
  bool                   rejectTarget;
  bool                   failPositionRead;
  bool                   ignoreHalt;
  bool                   isMissing;
  bool                   fakeEnableAck;
  bool                   dropStop;
  bool                   failStatusRead;
  bool                   wrongMode;
  uint32_t               commandCount;
} TestDriveTypeDef;

static TestDriveTypeDef s_drives[TEST_NODE_COUNT];
static uint32_t         s_tick;
static uint32_t         s_checks;

/** @brief 用具名断言输出精确失败条件。 */
static void Test_Check(bool condition, const char *pName)
{
  s_checks++;
  if (!condition)
  {
    (void) fprintf(stderr, "FAIL: %s\n", pName);
    exit(EXIT_FAILURE);
  }
}

/** @brief 初始化每个场景，模拟独立上电而不沿用上一个测试的历史。 */
static void Test_Reset(void)
{
  uint8_t index = 0U;
  (void) memset(s_drives, 0, sizeof(s_drives));
  (void) memset((void *) g_motorStartupDebug, 0, sizeof(g_motorStartupDebug));
  s_tick = 0U;
  for (index = 0U; index < TEST_NODE_COUNT; index++)
  {
    s_drives[index].node.nodeId = (uint8_t) (index + 1U);
    s_drives[index].node.isRegistered = true;
    s_drives[index].node.hasHeartbeat = true;
    s_drives[index].node.isOnline = true;
    s_drives[index].node.heartbeatCount = 1U;
    s_drives[index].node.nmtState = MID_CANOPEN_NMT_STATE_PRE_OPERATIONAL;
    s_drives[index].statusWord = TEST_DISABLED_STATUS;
#if CHASSIS_LINK_POSITION_TEST_ACTIVE
    s_drives[index].acceleration = 3855U;
    s_drives[index].deceleration = 4000U;
#endif
    Test_Check(APP_MotorStartup_AddNode((uint8_t) (index + 1U)) == MID_CANOPEN_OK, "register node");
  }
}

/** @brief 推进模拟时钟与独立心跳，并执行原始启动状态机。 */
static void Test_Run(uint32_t loops)
{
  uint32_t loop = 0U;
  uint8_t  index = 0U;
  for (loop = 0U; loop < loops; loop++)
  {
    s_tick += TEST_TICK_MS;
    for (index = 0U; index < TEST_NODE_COUNT; index++)
    {
      if (!s_drives[index].isMissing && ((s_tick % TEST_HEARTBEAT_MS) == 0U))
      {
        s_drives[index].node.hasHeartbeat = true;
        s_drives[index].node.isOnline = true;
        s_drives[index].node.heartbeatCount++;
      }
    }
    APP_MotorStartup_Process();
  }
}

uint32_t HAL_GetTick(void)
{
  return s_tick;
}
MID_CANopenNodeTypeDef *MID_CANopen_GetNode(uint8_t id)
{
  return ((id >= 1U) && (id <= TEST_NODE_COUNT)) ? &s_drives[id - 1U].node : NULL;
}
bool MID_CANopen_IsNodeOnline(uint8_t id)
{
  return ((id >= 1U) && (id <= TEST_NODE_COUNT)) && !s_drives[id - 1U].isMissing;
}
MID_CANopenStatusTypeDef MID_CANopen_Nmt_Control(uint8_t id, MID_CANopenNmtCommandTypeDef command)
{
  TestDriveTypeDef *pDrive = &s_drives[id - 1U];
  (void) command;
  if (pDrive->isMissing)
  {
    return MID_CANOPEN_TIMEOUT;
  }
  pDrive->commandCount++;
  pDrive->node.nmtState = MID_CANOPEN_NMT_STATE_OPERATIONAL;
  return MID_CANOPEN_OK;
}
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteU16(uint8_t id, uint16_t object, uint8_t sub, uint16_t value)
{
  TestDriveTypeDef *pDrive = &s_drives[id - 1U];
  (void) sub;
  if (pDrive->isMissing)
  {
    return MID_CANOPEN_TIMEOUT;
  }
  pDrive->commandCount++;
  if (object == MID_CANOPEN_OD_CONTROL_WORD)
  {
    if (value == MID_LISAN_EC_CONTROL_WORD_MOTOR_FREE)
    {
      pDrive->statusWord = TEST_READY_STATUS;
    }
    if (value == MID_LISAN_EC_CONTROL_WORD_MOTOR_LOCK)
    {
      pDrive->statusWord = TEST_SWITCHED_STATUS;
    }
    if (((value & 0x000FU) == MID_LISAN_EC_CONTROL_WORD_ENABLE_OPERATION) && !pDrive->fakeEnableAck)
    {
      pDrive->statusWord = TEST_ENABLED_STATUS;
    }
    if ((value & MID_LISAN_EC_CONTROL_WORD_NEW_SETPOINT) != 0U)
    {
      pDrive->triggerCount++;
      if (!pDrive->freezePosition)
      {
        pDrive->actualPosition += pDrive->targetPosition;
      }
      pDrive->statusWord |= MID_LISAN_EC_STATUS_WORD_TARGET_REACHED;
      if (pDrive->loseTriggerAck)
      {
        return MID_CANOPEN_TIMEOUT;
      }
    }
    if ((value == MID_LISAN_EC_CONTROL_WORD_HALT) && !pDrive->ignoreHalt)
    {
      pDrive->actualVelocity = 0;
    }
  }
  return MID_CANOPEN_OK;
}
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteI8(uint8_t id, uint16_t object, uint8_t sub, int8_t value)
{
  TestDriveTypeDef *pDrive = &s_drives[id - 1U];
  (void) object;
  (void) sub;
  if (pDrive->isMissing)
  {
    return MID_CANOPEN_TIMEOUT;
  }
  pDrive->commandCount++;
  if (!pDrive->wrongMode)
  {
    pDrive->mode = value;
  }
  return MID_CANOPEN_OK;
}
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteU32(uint8_t id, uint16_t object, uint8_t sub, uint32_t value)
{
  TestDriveTypeDef *pDrive = &s_drives[id - 1U];
  (void) sub;
  if (pDrive->isMissing)
  {
    return MID_CANOPEN_TIMEOUT;
  }
  pDrive->commandCount++;
  if (object == MID_CANOPEN_OD_PROFILE_ACCELERATION)
  {
    pDrive->rampWriteCount++;
    pDrive->acceleration = value;
  }
  if (object == MID_CANOPEN_OD_PROFILE_DECELERATION)
  {
    pDrive->rampWriteCount++;
    pDrive->deceleration = value;
  }
  if (object == MID_CANOPEN_OD_PROFILE_VELOCITY)
  {
    pDrive->profileSpeed = value;
  }
  return MID_CANOPEN_OK;
}
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteI32(uint8_t id, uint16_t object, uint8_t sub, int32_t value)
{
  TestDriveTypeDef *pDrive = &s_drives[id - 1U];
  (void) object;
  (void) sub;
  if (pDrive->isMissing || (pDrive->dropStop && (value == 0)))
  {
    return MID_CANOPEN_TIMEOUT;
  }
  pDrive->commandCount++;
  if (object == MID_CANOPEN_OD_TARGET_POSITION)
  {
    if (pDrive->rejectTarget && (value != 0))
    {
      return MID_CANOPEN_ABORTED;
    }
    pDrive->targetPosition = value;
    return MID_CANOPEN_OK;
  }
  pDrive->velocityWriteCount++;
  pDrive->targetVelocity = value;
  pDrive->actualVelocity = value;
  return MID_CANOPEN_OK;
}
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadU16(uint8_t id, uint16_t object, uint8_t sub, uint16_t *pValue)
{
  TestDriveTypeDef *pDrive = &s_drives[id - 1U];
  (void) sub;
  if (pDrive->isMissing || (pDrive->failStatusRead && (object == MID_CANOPEN_OD_STATUS_WORD)))
  {
    return MID_CANOPEN_TIMEOUT;
  }
  *pValue = (object == MID_CANOPEN_OD_STATUS_WORD) ? pDrive->statusWord : pDrive->errorCode;
  return MID_CANOPEN_OK;
}
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadI8(uint8_t id, uint16_t object, uint8_t sub, int8_t *pValue)
{
  (void) object;
  (void) sub;
  if (s_drives[id - 1U].isMissing)
  {
    return MID_CANOPEN_TIMEOUT;
  }
  *pValue = s_drives[id - 1U].mode;
  return MID_CANOPEN_OK;
}
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadU32(uint8_t id, uint16_t object, uint8_t sub, uint32_t *pValue)
{
  TestDriveTypeDef *pDrive = &s_drives[id - 1U];
  (void) sub;
  if (pDrive->isMissing)
  {
    return MID_CANOPEN_TIMEOUT;
  }
  *pValue = (object == MID_CANOPEN_OD_PROFILE_ACCELERATION) ? pDrive->acceleration : pDrive->deceleration;
  if (object == MID_CANOPEN_OD_PROFILE_VELOCITY)
  {
    *pValue = pDrive->profileSpeed;
  }
  return MID_CANOPEN_OK;
}
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadI32(uint8_t id, uint16_t object, uint8_t sub, int32_t *pValue)
{
  TestDriveTypeDef *pDrive = &s_drives[id - 1U];
  (void) sub;
  if (pDrive->isMissing)
  {
    return MID_CANOPEN_TIMEOUT;
  }
  *pValue = (object == MID_CANOPEN_OD_TARGET_VELOCITY) ? pDrive->targetVelocity : pDrive->actualVelocity;
  if (object == MID_CANOPEN_OD_TARGET_POSITION)
  {
    *pValue = pDrive->targetPosition;
  }
  if (object == MID_CANOPEN_OD_POSITION_ACTUAL_VALUE)
  {
    if (pDrive->failPositionRead)
    {
      return MID_CANOPEN_TIMEOUT;
    }
    *pValue = pDrive->actualPosition;
  }
  return MID_CANOPEN_OK;
}

MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteI16(uint8_t id, uint16_t object, uint8_t sub, int16_t value)
{
  (void) id;
  (void) object;
  (void) sub;
  (void) value;
  return MID_CANOPEN_INVALID_PARAM;
}
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteU8(uint8_t id, uint16_t object, uint8_t sub, uint8_t value)
{
  (void) id;
  (void) object;
  (void) sub;
  (void) value;
  return MID_CANOPEN_INVALID_PARAM;
}
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadI16(uint8_t id, uint16_t object, uint8_t sub, int16_t *pValue)
{
  (void) id;
  (void) object;
  (void) sub;
  (void) pValue;
  return MID_CANOPEN_INVALID_PARAM;
}

int main(void)
{
  uint8_t  step = 0U;
  uint32_t commandCount = 0U;
  Test_Reset();
  Test_Run(TEST_LOOPS);
  Test_Check(APP_MotorStartup_IsReady(1U) && APP_MotorStartup_IsReady(2U), "both nodes enabled");
  for (step = 0U; step < APP_MOTOR_STEP_PROFILE_SPEED; step++)
  {
    Test_Check(g_motorStartupDebug[0].firstSteps[step].result == APP_MOTOR_RESULT_VERIFIED,
               "every first startup step is retained");
  }
  Test_Check(g_motorStartupDebug[0].firstSteps[APP_MOTOR_STEP_SHUTDOWN].statusWord == TEST_READY_STATUS,
             "shutdown has its own state snapshot");
  Test_Check(g_motorStartupDebug[0].firstSteps[APP_MOTOR_STEP_ENABLE_OPERATION].statusWord == TEST_ENABLED_STATUS,
             "enable has its own state snapshot");
  Test_Check(APP_MotorStartup_RequestVelocity(1U, TEST_SPEED_PPS) == MID_CANOPEN_OK, "queue start");
  Test_Check(APP_MotorStartup_RequestVelocity(1U, TEST_SPEED_PPS) == MID_CANOPEN_BUSY,
             "pending command cannot be replaced");
  Test_Run(TEST_LOOPS);
  Test_Check(APP_MotorStartup_GetMotionResult(1U) == APP_MOTOR_RESULT_VERIFIED, "actual start verified");
  s_drives[0].dropStop = true;
  Test_Check(APP_MotorStartup_RequestVelocity(1U, 0) == MID_CANOPEN_OK, "queue failed stop");
  Test_Run(TEST_LOOPS);
  Test_Check(APP_MotorStartup_GetMotionResult(1U) == APP_MOTOR_RESULT_FAILED, "stop timeout is not success");
  Test_Check(g_motorStartupDebug[0].firstStop.actualVelocity == TEST_SPEED_PPS,
             "failed stop captures ongoing rotation");
  s_drives[0].dropStop = false;
  Test_Check(APP_MotorStartup_RequestVelocity(1U, 0) == MID_CANOPEN_OK, "queue stop retry");
  Test_Run(TEST_LOOPS);
  Test_Check(APP_MotorStartup_GetMotionResult(1U) == APP_MOTOR_RESULT_VERIFIED, "stop retry confirmed");
  Test_Check(g_motorStartupDebug[0].firstStop.result == APP_MOTOR_RESULT_FAILED, "retry preserves first stop failure");

  Test_Reset();
  s_drives[0].fakeEnableAck = true;
  Test_Run(TEST_LOOPS);
  Test_Check(!APP_MotorStartup_IsReady(1U) && APP_MotorStartup_IsReady(2U), "ACK without state change is not ready");
  Test_Check(g_motorStartupDebug[0].firstSteps[APP_MOTOR_STEP_ENABLE_OPERATION].commandAccepted,
             "communication ACK remains separate from execution");
  Test_Check(!g_motorStartupDebug[0].firstSteps[APP_MOTOR_STEP_ENABLE_OPERATION].effectConfirmed,
             "fake ACK execution failure is retained");
  Test_Check(g_motorStartupDebug[0].lastFailure.result == APP_MOTOR_RESULT_FAILED,
             "latest failure snapshot is complete");
  Test_Check(g_motorStartupDebug[0].lastFailure.step == APP_MOTOR_STEP_ENABLE_OPERATION,
             "failed step preserved across retry");
  Test_Check(g_motorStartupDebug[0].lastFailure.commandAccepted && !g_motorStartupDebug[0].lastFailure.effectConfirmed,
             "failure distinguishes ACK from effect");
  Test_Check(g_motorStartupDebug[0].failureCount > 0U, "failure counter updated");
  commandCount = s_drives[1].commandCount;
  Test_Run(TEST_LOOPS);
  Test_Check(s_drives[1].commandCount == commandCount, "failed peer does not reinitialize healthy wheel");
  s_drives[0].fakeEnableAck = false;
  Test_Run(TEST_LOOPS);
  Test_Check(APP_MotorStartup_IsReady(1U), "recovered node eventually enables");
  Test_Check(g_motorStartupDebug[0].lastFailure.result == APP_MOTOR_RESULT_FAILED &&
                 g_motorStartupDebug[0].lastFailure.step == APP_MOTOR_STEP_ENABLE_OPERATION,
             "successful recovery does not erase last failure");
  for (step = 0U; step < APP_MOTOR_STEP_PROFILE_SPEED; step++)
  {
    Test_Check(g_motorStartupDebug[0].latestSteps[step].result == APP_MOTOR_RESULT_VERIFIED,
               "successful retry keeps every command snapshot");
  }
  Test_Check(g_motorStartupDebug[0].firstSteps[APP_MOTOR_STEP_ENABLE_OPERATION].result == APP_MOTOR_RESULT_FAILED,
             "successful retry does not overwrite original failure");

  Test_Reset();
  s_drives[0].wrongMode = true;
  Test_Run(TEST_LOOPS);
  Test_Check(g_motorStartupDebug[0].firstSteps[APP_MOTOR_STEP_MODE].result == APP_MOTOR_RESULT_FAILED,
             "mode readback mismatch detected");
  Test_Check(g_motorStartupDebug[0].firstSteps[APP_MOTOR_STEP_ENABLE_OPERATION].result == APP_MOTOR_RESULT_NOT_RUN,
             "later enable is not sent after mismatch");
  Test_Check(g_motorStartupDebug[0].attemptCount > 1U, "failed node retries independently");

  Test_Reset();
  s_drives[0].failStatusRead = true;
  Test_Run(TEST_LOOPS);
  Test_Check(!APP_MotorStartup_IsReady(1U), "failed status read never becomes ready");
  Test_Check(g_motorStartupDebug[0].firstSteps[APP_MOTOR_STEP_NMT_START].statusReadStatus == MID_CANOPEN_TIMEOUT,
             "read timeout recorded explicitly");

  Test_Reset();
  s_drives[0].statusWord = TEST_FAULT_STATUS;
  s_drives[0].errorCode = TEST_ERROR_CODE;
  Test_Run(TEST_LOOPS);
  Test_Check(g_motorStartupDebug[0].firstSteps[APP_MOTOR_STEP_NMT_START].errorCode == TEST_ERROR_CODE,
             "fault code retained");
  Test_Check(!APP_MotorStartup_IsReady(1U), "faulted drive never marked ready");

  Test_Reset();
  s_drives[0].isMissing = true;
  s_drives[0].node.hasHeartbeat = false;
  Test_Run(TEST_LOOPS);
  Test_Check(g_motorStartupDebug[0].firstSteps[APP_MOTOR_STEP_WAIT_HEARTBEAT].result == APP_MOTOR_RESULT_FAILED,
             "absent node boot timeout retained");
  Test_Check(APP_MotorStartup_IsReady(2U), "absent peer does not block other wheel");
  (void) printf(
      "PASS: %lu checks across startup, fake ACK, read mismatch, timeout, fault, missing node and stop retry.\n",
      (unsigned long) s_checks);
  return EXIT_SUCCESS;
}
