/**
 ******************************************************************************
 * @file    app_motor_startup.c
 * @brief   逐命令启动状态机，分开记录通信应答与驱动器执行结果。
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

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include "chassis_link_config.h"

/** @defgroup APP_MOTOR_STARTUP_Phase 启动命令子阶段
 * @{
 */
typedef enum
{
  APP_MOTOR_PHASE_WAIT_BOOT = 0U,
  APP_MOTOR_PHASE_SEND,
  APP_MOTOR_PHASE_STATUS,
  APP_MOTOR_PHASE_VERIFY,
  APP_MOTOR_PHASE_VELOCITY,
  APP_MOTOR_PHASE_ERROR,
  APP_MOTOR_PHASE_EVALUATE,
  APP_MOTOR_PHASE_RETRY,
  APP_MOTOR_PHASE_IDLE
} APP_MotorStartupPhaseTypeDef;
/** @} */

typedef struct
{
  APP_MotorStartupPhaseTypeDef phase;
  APP_MotorCommandDebugTypeDef record;
  uint32_t                     nextTick;
  uint32_t                     heartbeatBaseline;
  bool                         commandFailed;
  bool                         isMotion;
} APP_MotorStartupContextTypeDef;

volatile APP_MotorStartupDebugTypeDef g_motorStartupDebug[CANOPEN_MAX_NODE_COUNT];
static APP_MotorStartupContextTypeDef s_contexts[CANOPEN_MAX_NODE_COUNT];
static uint8_t                        s_nextNode = 0U;
static uint32_t                       s_sequence = 0U;

static uint8_t                  APP_MotorStartup_FindNode(uint8_t nodeId);
static void                     APP_MotorStartup_Begin(uint8_t index, APP_MotorStartupStepTypeDef step, int32_t value);
static void                     APP_MotorStartup_Publish(uint8_t index);
static void                     APP_MotorStartup_Finish(uint8_t index, bool confirmed);
static MID_CANopenStatusTypeDef APP_MotorStartup_Send(uint8_t index);
static MID_CANopenStatusTypeDef APP_MotorStartup_Verify(uint8_t index);
static bool                     APP_MotorStartup_HasEffect(uint8_t index);
static void                     APP_MotorStartup_RunNode(uint8_t index);

/**
 * @brief  按节点地址查找诊断槽位。
 * @note   未注册地址返回节点容量上限，调用方必须先检查再访问数组。
 */
static uint8_t APP_MotorStartup_FindNode(uint8_t nodeId)
{
  uint8_t index = 0U;
  for (index = 0U; index < CANOPEN_MAX_NODE_COUNT; index++)
  {
    if (g_motorStartupDebug[index].isRegistered && (g_motorStartupDebug[index].nodeId == nodeId))
    {
      return index;
    }
  }
  return CANOPEN_MAX_NODE_COUNT;
}

/**
 * @brief  为下一条命令建立全部字段均有有效性标志的记录。
 * @note   读回状态初始为 NOT_READY，避免未读取的零值被误认为有效数据。
 */
static void APP_MotorStartup_Begin(uint8_t index, APP_MotorStartupStepTypeDef step, int32_t value)
{
  APP_MotorStartupContextTypeDef *pContext = &s_contexts[index];
  MID_CANopenNodeTypeDef         *pNode = MID_CANopen_GetNode(g_motorStartupDebug[index].nodeId);
  (void) memset(&pContext->record, 0, sizeof(pContext->record));
  pContext->record.step = step;
  pContext->record.result = APP_MOTOR_RESULT_PENDING;
  pContext->record.sequence = ++s_sequence;
  pContext->record.startTick = HAL_GetTick();
  pContext->record.commandValue = value;
  pContext->record.commandStatus = MID_CANOPEN_NOT_READY;
  pContext->record.statusReadStatus = MID_CANOPEN_NOT_READY;
  pContext->record.verifyReadStatus = MID_CANOPEN_NOT_READY;
  pContext->record.velocityReadStatus = MID_CANOPEN_NOT_READY;
  pContext->record.errorReadStatus = MID_CANOPEN_NOT_READY;
  pContext->record.nmtState = MID_CANOPEN_NMT_STATE_UNKNOWN;
  pContext->commandFailed = false;
  pContext->isMotion = (step == APP_MOTOR_STEP_MOTION);
  pContext->heartbeatBaseline = (pNode != NULL) ? pNode->heartbeatCount : 0U;
  pContext->nextTick = HAL_GetTick();
  pContext->phase = (step == APP_MOTOR_STEP_WAIT_HEARTBEAT) ? APP_MOTOR_PHASE_WAIT_BOOT : APP_MOTOR_PHASE_SEND;
  g_motorStartupDebug[index].currentStep = step;
  APP_MotorStartup_Publish(index);
}

/**
 * @brief  把当前阶段发布到可由调试器直接读取的全局结构体。
 * @note   首次历史保存在 firstSteps，当前尝试更新 latestSteps；暂停在发布函数内部时记录可能尚未复制完整。
 */
static void APP_MotorStartup_Publish(uint8_t index)
{
  APP_MotorCommandDebugTypeDef *pRecord = &s_contexts[index].record;
  g_motorStartupDebug[index].latest = *pRecord;
  if (pRecord->step < APP_MOTOR_STEP_COUNT)
  {
    g_motorStartupDebug[index].latestSteps[pRecord->step] = *pRecord;
  }
  if ((g_motorStartupDebug[index].attemptCount == 1U) && (pRecord->step < APP_MOTOR_STEP_COUNT))
  {
    g_motorStartupDebug[index].firstSteps[pRecord->step] = *pRecord;
  }
}

/**
 * @brief  完成注册节点的启动尝试或运动验证，成功后才推进控制状态机。
 * @note   首次历史保留在 firstSteps，最近一次失败锁存在 lastFailure，成功或重试不清除失败快照。
 *         故障仅记录，不自动清除驱动器报警。
 */
static void APP_MotorStartup_Finish(uint8_t index, bool confirmed)
{
  APP_MotorStartupContextTypeDef *pContext = &s_contexts[index];
  APP_MotorStartupStepTypeDef     step = pContext->record.step;
  pContext->record.endTick = HAL_GetTick();
  pContext->record.effectConfirmed = confirmed;
  pContext->record.result = confirmed ? APP_MOTOR_RESULT_VERIFIED : APP_MOTOR_RESULT_FAILED;
  APP_MotorStartup_Publish(index);
  if (!confirmed)
  {
    g_motorStartupDebug[index].lastFailure = pContext->record;
    g_motorStartupDebug[index].failureCount++;
  }
  if (pContext->isMotion)
  {
    if ((pContext->record.commandValue != 0) &&
        (g_motorStartupDebug[index].firstStart.result == APP_MOTOR_RESULT_NOT_RUN))
    {
      g_motorStartupDebug[index].firstStart = pContext->record;
    }
    if ((pContext->record.commandValue == 0) &&
        (g_motorStartupDebug[index].firstStop.result == APP_MOTOR_RESULT_NOT_RUN))
    {
      g_motorStartupDebug[index].firstStop = pContext->record;
    }
    pContext->phase = APP_MOTOR_PHASE_IDLE;
    return;
  }
  if (!confirmed)
  {
    g_motorStartupDebug[index].failedStep = step;
    g_motorStartupDebug[index].firstAttemptFinished = true;
    pContext->nextTick = HAL_GetTick();
    pContext->phase = APP_MOTOR_PHASE_RETRY;
    return;
  }
  if (step == APP_MOTOR_STEP_ENABLE_OPERATION)
  {
    g_motorStartupDebug[index].isReady = true;
    g_motorStartupDebug[index].firstAttemptFinished = true;
    pContext->phase = APP_MOTOR_PHASE_IDLE;
    return;
  }
#if CHASSIS_LINK_POSITION_TEST_ACTIVE
  if (step == APP_MOTOR_STEP_DECELERATION)
  {
    step = APP_MOTOR_STEP_PROFILE_SPEED;
  }
  else if (step == APP_MOTOR_STEP_PROFILE_SPEED)
  {
    step = APP_MOTOR_STEP_ZERO_VELOCITY;
  }
  else
#endif
  {
    step = (APP_MotorStartupStepTypeDef) (step + 1U);
  }
#if !CHASSIS_LINK_POSITION_TEST_ACTIVE
  g_motorStartupDebug[index].latestSteps[APP_MOTOR_STEP_PROFILE_SPEED].result = APP_MOTOR_RESULT_SKIPPED;
  if (g_motorStartupDebug[index].attemptCount == 1U)
  {
    g_motorStartupDebug[index].firstSteps[APP_MOTOR_STEP_PROFILE_SPEED].result = APP_MOTOR_RESULT_SKIPPED;
  }
#endif
#if (CHASSIS_LINK_ENABLE_PROFILE_RAMP == 0U)
  if (step == APP_MOTOR_STEP_ACCELERATION)
  {
    g_motorStartupDebug[index].latestSteps[APP_MOTOR_STEP_ACCELERATION].result = APP_MOTOR_RESULT_SKIPPED;
    g_motorStartupDebug[index].latestSteps[APP_MOTOR_STEP_DECELERATION].result = APP_MOTOR_RESULT_SKIPPED;
    if (g_motorStartupDebug[index].attemptCount == 1U)
    {
      g_motorStartupDebug[index].firstSteps[APP_MOTOR_STEP_ACCELERATION].result = APP_MOTOR_RESULT_SKIPPED;
      g_motorStartupDebug[index].firstSteps[APP_MOTOR_STEP_DECELERATION].result = APP_MOTOR_RESULT_SKIPPED;
    }
#if CHASSIS_LINK_POSITION_TEST_ACTIVE
    step = APP_MOTOR_STEP_PROFILE_SPEED;
#else
    step = APP_MOTOR_STEP_ZERO_VELOCITY;
#endif
  }
#endif
  APP_MotorStartup_Begin(index, step, 0);
}

#if CHASSIS_LINK_POSITION_TEST_ACTIVE
/**
 * @brief 读取当前加减速度供位置测试使用，不重写驱动器参数。
 * @note 读到零或超出调试记录可表示的正值范围时失败；随后仍要再次读回确认。
 */
static MID_CANopenStatusTypeDef APP_MotorStartup_ReadExistingRamp(uint8_t nodeId, APP_MotorCommandDebugTypeDef *pRecord)
{
  uint32_t                 value = 0U;
  MID_CANopenStatusTypeDef status =
      MID_CANopen_Sdo_ReadU32(nodeId, pRecord->objectIndex, MID_CANOPEN_OD_SUB_INDEX_0, &value);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }
  pRecord->readbackValue = (int32_t) value;
  if ((value == 0U) || (value > INT32_MAX))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }
  pRecord->commandValue = (int32_t) value;
  return MID_CANOPEN_OK;
}
#endif

/**
 * @brief  执行当前启动参数事务或控制命令，并记录对象索引与参数值。
 * @note   NMT 无 SDO 应答，通信结果只代表已入发送邮箱，生效由新心跳及状态字验证。
 *         位置测试的加减速度只读取现有值，不发送参数写入。
 */
static MID_CANopenStatusTypeDef APP_MotorStartup_Send(uint8_t index)
{
  APP_MotorCommandDebugTypeDef *pRecord = &s_contexts[index].record;
  uint8_t                       nodeId = g_motorStartupDebug[index].nodeId;
  switch (pRecord->step)
  {
    case APP_MOTOR_STEP_NMT_START:
      pRecord->commandValue = MID_CANOPEN_NMT_START_REMOTE_NODE;
      return MID_CANopen_Nmt_Control(nodeId, MID_CANOPEN_NMT_START_REMOTE_NODE);
    case APP_MOTOR_STEP_MODE:
      pRecord->objectIndex = MID_CANOPEN_OD_MODE_OF_OPERATION;
#if CHASSIS_LINK_POSITION_TEST_ACTIVE
      pRecord->commandValue = MID_LISAN_EC_MODE_PROFILE_POSITION;
#else
      pRecord->commandValue = MID_LISAN_EC_MODE_PROFILE_VELOCITY;
#endif
      return MID_LisanEc_SetOperationMode(nodeId, (MID_LisanEcModeTypeDef) pRecord->commandValue);
    case APP_MOTOR_STEP_ACCELERATION:
      pRecord->objectIndex = MID_CANOPEN_OD_PROFILE_ACCELERATION;
#if CHASSIS_LINK_POSITION_TEST_ACTIVE
      return APP_MotorStartup_ReadExistingRamp(nodeId, pRecord);
#else
      pRecord->commandValue = CHASSIS_LINK_PROFILE_ACCELERATION_PPS2;
#endif
      return MID_LisanEc_SetProfileAcceleration(nodeId, (uint32_t) pRecord->commandValue);
    case APP_MOTOR_STEP_DECELERATION:
      pRecord->objectIndex = MID_CANOPEN_OD_PROFILE_DECELERATION;
#if CHASSIS_LINK_POSITION_TEST_ACTIVE
      return APP_MotorStartup_ReadExistingRamp(nodeId, pRecord);
#else
      pRecord->commandValue = CHASSIS_LINK_PROFILE_DECELERATION_PPS2;
#endif
      return MID_LisanEc_SetProfileDeceleration(nodeId, (uint32_t) pRecord->commandValue);
    case APP_MOTOR_STEP_PROFILE_SPEED:
      pRecord->objectIndex = MID_CANOPEN_OD_PROFILE_VELOCITY;
      pRecord->commandValue = CHASSIS_LINK_POSITION_SPEED_PPS;
      return MID_CANopen_Sdo_WriteU32(nodeId, pRecord->objectIndex, 0U, (uint32_t) pRecord->commandValue);
    case APP_MOTOR_STEP_ZERO_VELOCITY:
#if CHASSIS_LINK_POSITION_TEST_ACTIVE
      pRecord->objectIndex = MID_CANOPEN_OD_TARGET_POSITION;
      return MID_LisanEc_SetTargetPosition(nodeId, 0);
#endif
    case APP_MOTOR_STEP_MOTION:
      pRecord->objectIndex = MID_CANOPEN_OD_TARGET_VELOCITY;
      return MID_LisanEc_SetTargetVelocity(nodeId, pRecord->commandValue);
    case APP_MOTOR_STEP_SHUTDOWN:
      pRecord->commandValue = MID_LISAN_EC_CONTROL_WORD_MOTOR_FREE;
      break;
    case APP_MOTOR_STEP_SWITCH_ON:
      pRecord->commandValue = MID_LISAN_EC_CONTROL_WORD_MOTOR_LOCK;
      break;
    case APP_MOTOR_STEP_ENABLE_OPERATION:
      pRecord->commandValue = MID_LISAN_EC_CONTROL_WORD_ENABLE_OPERATION;
#if CHASSIS_LINK_POSITION_TEST_ACTIVE
      pRecord->commandValue |= MID_LISAN_EC_CONTROL_WORD_RELATIVE_POSITION;
#endif
      break;
    default:
      return MID_CANOPEN_INVALID_PARAM;
  }
  pRecord->objectIndex = MID_CANOPEN_OD_CONTROL_WORD;
  return MID_LisanEc_WriteControlWord(nodeId, (uint16_t) pRecord->commandValue);
}

/**
 * @brief  读回当前命令对应的参数或工作模式，控制字通过实际状态而非回显判断。
 * @note   写模式 6060 后读取 6061，其他参数读回原对象；所有读回保留独立返回值。
 */
static MID_CANopenStatusTypeDef APP_MotorStartup_Verify(uint8_t index)
{
  APP_MotorCommandDebugTypeDef *pRecord = &s_contexts[index].record;
  uint8_t                       nodeId = g_motorStartupDebug[index].nodeId;
  MID_CANopenStatusTypeDef      status = MID_CANOPEN_OK;
  MID_LisanEcModeTypeDef        mode;
  uint32_t                      unsignedValue = 0U;
  if (pRecord->step == APP_MOTOR_STEP_MODE)
  {
    status = MID_LisanEc_ReadOperationMode(nodeId, &mode);
    if (status == MID_CANOPEN_OK)
    {
      pRecord->readbackValue = (int32_t) mode;
    }
  }
  else if ((pRecord->step == APP_MOTOR_STEP_ACCELERATION) || (pRecord->step == APP_MOTOR_STEP_DECELERATION) ||
           (pRecord->step == APP_MOTOR_STEP_PROFILE_SPEED))
  {
    status = MID_CANopen_Sdo_ReadU32(nodeId, pRecord->objectIndex, MID_CANOPEN_OD_SUB_INDEX_0, &unsignedValue);
    if (status == MID_CANOPEN_OK)
    {
      pRecord->readbackValue = (int32_t) unsignedValue;
    }
  }
  else if ((pRecord->step == APP_MOTOR_STEP_ZERO_VELOCITY) || (pRecord->step == APP_MOTOR_STEP_MOTION))
  {
    status = MID_CANopen_Sdo_ReadI32(nodeId, pRecord->objectIndex, MID_CANOPEN_OD_SUB_INDEX_0, &pRecord->readbackValue);
  }
  else
  {
    status = pRecord->statusReadStatus;
    pRecord->readbackValue = pRecord->statusWord;
  }
  return status;
}

/**
 * @brief  根据新心跳、实际驱动状态和读回参数判断命令已生效。
 * @note   已使能不等于转动；运动步骤必须同时验证目标与实际速度，停机使用速度零容差。
 */
static bool APP_MotorStartup_HasEffect(uint8_t index)
{
  APP_MotorStartupContextTypeDef *pContext = &s_contexts[index];
  APP_MotorCommandDebugTypeDef   *pRecord = &pContext->record;
  int64_t                         delta = (int64_t) pRecord->actualVelocity - pRecord->commandValue;
  if ((pRecord->statusReadStatus != MID_CANOPEN_OK) || (pRecord->verifyReadStatus != MID_CANOPEN_OK) ||
      ((pRecord->statusWord & MID_LISAN_EC_STATUS_WORD_FAULT) != 0U))
  {
    return false;
  }
  switch (pRecord->step)
  {
    case APP_MOTOR_STEP_NMT_START:
      return (pRecord->heartbeatCount != pContext->heartbeatBaseline) &&
             (pRecord->nmtState == MID_CANOPEN_NMT_STATE_OPERATIONAL) &&
             ((pRecord->statusWord & MID_LISAN_EC_STATUS_WORD_REMOTE) != 0U);
    case APP_MOTOR_STEP_SHUTDOWN:
      return pRecord->driveState == MID_LISAN_EC_STATE_READY_TO_SWITCH_ON;
    case APP_MOTOR_STEP_SWITCH_ON:
      return pRecord->driveState == MID_LISAN_EC_STATE_SWITCHED_ON;
    case APP_MOTOR_STEP_ENABLE_OPERATION:
      return (pRecord->driveState == MID_LISAN_EC_STATE_OPERATION_ENABLED) &&
             ((pRecord->statusWord & MID_LISAN_EC_STATUS_WORD_REMOTE) != 0U);
    case APP_MOTOR_STEP_MOTION:
      return (pRecord->readbackValue == pRecord->commandValue) &&
             (pRecord->driveState == MID_LISAN_EC_STATE_OPERATION_ENABLED) &&
             (pRecord->velocityReadStatus == MID_CANOPEN_OK) && (delta <= CHASSIS_LINK_MOTION_SPEED_TOLERANCE_PPS) &&
             (delta >= -CHASSIS_LINK_MOTION_SPEED_TOLERANCE_PPS);
    default:
      return pRecord->readbackValue == pRecord->commandValue;
  }
}

/**
 * @brief  每次仅执行一个发送或读取事务，等待状态切换时交还任务。
 * @note   阶段超时从命令应答后开始；命令失败仍采集诊断快照，但不继续下一个启动命令。
 */
static void APP_MotorStartup_RunNode(uint8_t index)
{
  APP_MotorStartupContextTypeDef *pContext = &s_contexts[index];
  APP_MotorCommandDebugTypeDef   *pRecord = &pContext->record;
  uint8_t                         nodeId = g_motorStartupDebug[index].nodeId;
  MID_CANopenNodeTypeDef         *pNode = MID_CANopen_GetNode(nodeId);
  MID_LisanEcStatusWordTypeDef    driveStatus;
  uint32_t                        currentTick = HAL_GetTick();
  bool                            isRead = false;
  bool                            isExistingRamp = false;
  uint8_t                         stepIndex = 0U;
  switch (pContext->phase)
  {
    case APP_MOTOR_PHASE_WAIT_BOOT:
      if ((pNode != NULL) && pNode->hasHeartbeat && MID_CANopen_IsNodeOnline(nodeId))
      {
        pRecord->heartbeatCount = pNode->heartbeatCount;
        pRecord->nmtState = pNode->nmtState;
        pRecord->verifyReadStatus = MID_CANOPEN_OK;
        APP_MotorStartup_Finish(index, true);
      }
      else if ((currentTick - pRecord->startTick) >= CHASSIS_LINK_BOOT_WAIT_MS)
      {
        pRecord->verifyReadStatus = MID_CANOPEN_TIMEOUT;
        APP_MotorStartup_Finish(index, false);
      }
      break;
    case APP_MOTOR_PHASE_SEND:
      pRecord->commandStatus = APP_MotorStartup_Send(index);
#if CHASSIS_LINK_POSITION_TEST_ACTIVE
      isExistingRamp = (pRecord->step == APP_MOTOR_STEP_ACCELERATION) || (pRecord->step == APP_MOTOR_STEP_DECELERATION);
#endif
      pRecord->commandAccepted =
          !isExistingRamp && (pRecord->step != APP_MOTOR_STEP_NMT_START) && (pRecord->commandStatus == MID_CANOPEN_OK);
      pRecord->commandAbortCode =
          ((pNode != NULL) && (pRecord->commandStatus == MID_CANOPEN_ABORTED)) ? pNode->lastSdoAbortCode : 0U;
      if (isExistingRamp)
      {
        pRecord->readAbortCode = pRecord->commandAbortCode;
        pRecord->commandAbortCode = 0U;
      }
      pContext->commandFailed = (pRecord->commandStatus != MID_CANOPEN_OK);
      pContext->nextTick = HAL_GetTick();
      pContext->phase = APP_MOTOR_PHASE_STATUS;
      break;
    case APP_MOTOR_PHASE_STATUS:
      isRead = true;
      pRecord->statusReadStatus = MID_LisanEc_ReadStatusWord(nodeId, &driveStatus);
      if (pRecord->statusReadStatus == MID_CANOPEN_OK)
      {
        pRecord->statusWord = driveStatus.raw;
        pRecord->driveState = MID_LisanEc_ParseState(driveStatus.raw);
      }
      pContext->phase = APP_MOTOR_PHASE_VERIFY;
      break;
    case APP_MOTOR_PHASE_VERIFY:
      isRead = true;
      pRecord->verifyReadStatus = APP_MotorStartup_Verify(index);
      pContext->phase = APP_MOTOR_PHASE_VELOCITY;
      break;
    case APP_MOTOR_PHASE_VELOCITY:
      isRead = true;
      pRecord->velocityReadStatus = MID_LisanEc_ReadVelocity(nodeId, &pRecord->actualVelocity);
      pContext->phase = APP_MOTOR_PHASE_ERROR;
      break;
    case APP_MOTOR_PHASE_ERROR:
      isRead = true;
      pRecord->errorReadStatus = MID_LisanEc_ReadErrorCode(nodeId, &pRecord->errorCode);
      pContext->phase = APP_MOTOR_PHASE_EVALUATE;
      break;
    case APP_MOTOR_PHASE_EVALUATE:
      if (pNode != NULL)
      {
        pRecord->heartbeatCount = pNode->heartbeatCount;
        pRecord->nmtState = pNode->nmtState;
      }
      if (!pContext->commandFailed && APP_MotorStartup_HasEffect(index))
      {
        APP_MotorStartup_Finish(index, true);
      }
      else if (pContext->commandFailed ||
               ((pRecord->statusReadStatus == MID_CANOPEN_OK) &&
                ((pRecord->statusWord & MID_LISAN_EC_STATUS_WORD_FAULT) != 0U)) ||
               ((currentTick - pContext->nextTick) >= CHASSIS_LINK_COMMAND_VERIFY_TIMEOUT_MS))
      {
        APP_MotorStartup_Finish(index, false);
      }
      else
      {
        pContext->phase = APP_MOTOR_PHASE_STATUS;
      }
      break;
    case APP_MOTOR_PHASE_RETRY:
      if ((currentTick - pContext->nextTick) >= CHASSIS_LINK_DRIVE_RETRY_PERIOD_MS)
      {
        APP_MotorCommandDebugTypeDef emptyRecord;
        (void) memset(&emptyRecord, 0, sizeof(emptyRecord));
        for (stepIndex = 0U; stepIndex < APP_MOTOR_STEP_COUNT; stepIndex++)
        {
          g_motorStartupDebug[index].latestSteps[stepIndex] = emptyRecord;
        }
        g_motorStartupDebug[index].attemptCount++;
        APP_MotorStartup_Begin(index, APP_MOTOR_STEP_WAIT_HEARTBEAT, 0);
      }
      break;
    default:
      break;
  }
  if (isRead && (pNode != NULL) && (pNode->lastSdoAbortCode != 0U))
  {
    pRecord->readAbortCode = pNode->lastSdoAbortCode;
  }
  APP_MotorStartup_Publish(index);
}

/**
 * @brief  注册独立启动上下文，节点地址需已在 CANopen 节点表中存在。
 * @note   重复注册不清历史；首次建立等待心跳步骤，配置命令由任务周期执行。
 */
MID_CANopenStatusTypeDef APP_MotorStartup_AddNode(uint8_t nodeId)
{
  uint8_t index = 0U;
  if (MID_CANopen_GetNode(nodeId) == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }
  if (APP_MotorStartup_FindNode(nodeId) < CANOPEN_MAX_NODE_COUNT)
  {
    return MID_CANOPEN_OK;
  }
  for (index = 0U; index < CANOPEN_MAX_NODE_COUNT; index++)
  {
    if (!g_motorStartupDebug[index].isRegistered)
    {
      g_motorStartupDebug[index].nodeId = nodeId;
      g_motorStartupDebug[index].isRegistered = true;
      g_motorStartupDebug[index].attemptCount = 1U;
      APP_MotorStartup_Begin(index, APP_MOTOR_STEP_WAIT_HEARTBEAT, 0);
      return MID_CANOPEN_OK;
    }
  }
  return MID_CANOPEN_ERROR;
}

/**
 * @brief  轮流推进各节点一次，保证单轮失败不会反复清零另一轮的速度。
 * @note   一次调用最多进行一个 SDO 事务；阻塞上限仍受 CANOPEN_SDO_TIMEOUT_MS 约束。
 */
void APP_MotorStartup_Process(void)
{
  uint8_t count = 0U;
  uint8_t index = 0U;
  for (count = 0U; count < CANOPEN_MAX_NODE_COUNT; count++)
  {
    index = s_nextNode;
    s_nextNode = (uint8_t) ((s_nextNode + 1U) % CANOPEN_MAX_NODE_COUNT);
    if (g_motorStartupDebug[index].isRegistered && (s_contexts[index].phase != APP_MOTOR_PHASE_IDLE))
    {
      APP_MotorStartup_RunNode(index);
      return;
    }
  }
}

/**
 * @brief  返回实际使能状态已验证的启动结果。
 * @note   未注册节点返回 false，单次 SDO 写入成功不设置此标志。
 */
bool APP_MotorStartup_IsReady(uint8_t nodeId)
{
  uint8_t index = APP_MotorStartup_FindNode(nodeId);
  return (index < CANOPEN_MAX_NODE_COUNT) && g_motorStartupDebug[index].isReady;
}

/**
 * @brief  排入一条目标速度命令，随后读回目标、实际速度、状态字和错误码。
 * @note   OK 仅表示请求入队；必须等待 GetMotionResult 返回 VERIFIED 才能判定运动或停止生效。
 */
MID_CANopenStatusTypeDef APP_MotorStartup_RequestVelocity(uint8_t nodeId, int32_t velocity)
{
  uint8_t index = APP_MotorStartup_FindNode(nodeId);
#if CHASSIS_LINK_POSITION_TEST_ACTIVE
  (void) velocity;
  (void) index;
  return MID_CANOPEN_INVALID_PARAM;
#endif
  if ((index >= CANOPEN_MAX_NODE_COUNT) || !g_motorStartupDebug[index].isReady)
  {
    return MID_CANOPEN_NOT_READY;
  }
  if (s_contexts[index].phase != APP_MOTOR_PHASE_IDLE)
  {
    return MID_CANOPEN_BUSY;
  }
  APP_MotorStartup_Begin(index, APP_MOTOR_STEP_MOTION, velocity);
  return MID_CANOPEN_OK;
}

/**
 * @brief  查询最近一次运动命令是否完成实际速度验证。
 * @note   未注册节点或尚未请求运动返回 NOT_RUN，读取失败返回 FAILED 而不沿用旧速度。
 */
APP_MotorCommandResultTypeDef APP_MotorStartup_GetMotionResult(uint8_t nodeId)
{
  uint8_t index = APP_MotorStartup_FindNode(nodeId);
  if ((index >= CANOPEN_MAX_NODE_COUNT) || (s_contexts[index].record.step != APP_MOTOR_STEP_MOTION))
  {
    return APP_MOTOR_RESULT_NOT_RUN;
  }
  return s_contexts[index].record.result;
}

volatile APP_PositionTestDebugTypeDef g_motorPositionDebug[CANOPEN_MAX_NODE_COUNT];

#if CHASSIS_LINK_POSITION_TEST_ACTIVE
static APP_PositionTestDebugTypeDef s_positionTests[CANOPEN_MAX_NODE_COUNT];
static uint8_t                      s_positionActive = CANOPEN_MAX_NODE_COUNT;
static uint32_t                     s_positionNextTick;

/**
 * @brief 建立本段位移记录，所有读取和写入状态初始化为未执行。
 * @note 第一段正向，第二段反向；各节点每次上电只执行这一对有限位移。
 */
static void APP_PositionTest_BeginMove(APP_PositionTestDebugTypeDef *pTest)
{
  APP_PositionMoveDebugTypeDef *pMove = &pTest->moves[pTest->moveIndex];
  uint8_t                       phase;
  (void) memset(pMove, 0, sizeof(*pMove));
  for (phase = 0U; phase < APP_POSITION_PHASE_COUNT; phase++)
  {
    pMove->operationStatus[phase] = MID_CANOPEN_NOT_READY;
  }
  pMove->result = APP_MOTOR_RESULT_PENDING;
  pMove->startTick = HAL_GetTick();
  pMove->relativePulses =
      (pTest->moveIndex == 0U) ? CHASSIS_LINK_POSITION_DISTANCE_PULSES : -CHASSIS_LINK_POSITION_DISTANCE_PULSES;
  pTest->phase = APP_POSITION_PHASE_MODE;
}

/**
 * @brief 锁存失败阶段并进入停止流程。
 * @note 不重试相对位移触发，避免应答丢失时重复走一段；停稳前不允许另一轮开始。
 */
static void APP_PositionTest_Fail(APP_PositionTestDebugTypeDef *pTest)
{
  APP_PositionMoveDebugTypeDef *pMove = &pTest->moves[pTest->moveIndex];
  pMove->failedPhase = pTest->phase;
  pMove->result = APP_MOTOR_RESULT_FAILED;
  pMove->endTick = HAL_GetTick();
  pTest->result = APP_MOTOR_RESULT_FAILED;
  pTest->phase = APP_POSITION_PHASE_HALT;
  s_positionNextTick = HAL_GetTick() - CHASSIS_LINK_DRIVE_RETRY_PERIOD_MS;
}
#endif

/**
 * @brief 对已就绪节点逐个执行正反各一次有限位移，并记录真实到位和停稳结果。
 * @note 每次至多一个 SDO 事务；触发前校验模式、相对目标和 bit4 清零应答。
 *       完成必须满足状态、位置误差、到位位和速度条件，任何失败均锁存并尝试 Halt。
 */
void APP_MotorStartup_PositionTestProcess(void)
{
#if CHASSIS_LINK_POSITION_TEST_ACTIVE
  APP_PositionTestDebugTypeDef *pTest;
  APP_PositionMoveDebugTypeDef *pMove;
  APP_PositionPhaseTypeDef      phase;
  MID_CANopenStatusTypeDef      status = MID_CANOPEN_OK;
  MID_CANopenNodeTypeDef       *pNode;
  MID_LisanEcStatusWordTypeDef  state;
  uint32_t                      now = HAL_GetTick();
  int64_t                       expected;
  int64_t                       error;
  bool                          isTransaction = true;
  uint8_t                       index;

  if (s_positionActive >= CANOPEN_MAX_NODE_COUNT)
  {
    for (index = 0U; index < CANOPEN_MAX_NODE_COUNT; index++)
    {
      if (g_motorStartupDebug[index].isRegistered && g_motorStartupDebug[index].isReady &&
          (s_positionTests[index].result == APP_MOTOR_RESULT_NOT_RUN))
      {
        s_positionActive = index;
        s_positionTests[index].nodeId = g_motorStartupDebug[index].nodeId;
        s_positionTests[index].result = APP_MOTOR_RESULT_PENDING;
        APP_PositionTest_BeginMove(&s_positionTests[index]);
        break;
      }
    }
    if (s_positionActive >= CANOPEN_MAX_NODE_COUNT)
    {
      return;
    }
  }
  index = s_positionActive;
  pTest = &s_positionTests[index];
  pMove = &pTest->moves[pTest->moveIndex];
  if ((pMove->result == APP_MOTOR_RESULT_PENDING) && ((now - pMove->startTick) >= CHASSIS_LINK_POSITION_TIMEOUT_MS))
  {
    APP_PositionTest_Fail(pTest);
  }
  phase = pTest->phase;
  switch (phase)
  {
    case APP_POSITION_PHASE_MODE:
      status = MID_LisanEc_ReadOperationMode(pTest->nodeId, &pMove->actualMode);
      if ((status == MID_CANOPEN_OK) && (pMove->actualMode != MID_LISAN_EC_MODE_PROFILE_POSITION))
      {
        APP_PositionTest_Fail(pTest);
      }
      break;
    case APP_POSITION_PHASE_BASELINE:
      status = MID_LisanEc_ReadPosition(pTest->nodeId, &pMove->startPosition);
      expected = (int64_t) pMove->startPosition + pMove->relativePulses;
      if ((status == MID_CANOPEN_OK) && ((expected > INT32_MAX) || (expected < INT32_MIN)))
      {
        APP_PositionTest_Fail(pTest);
      }
      else if (status == MID_CANOPEN_OK)
      {
        pMove->expectedPosition = (int32_t) expected;
      }
      break;
    case APP_POSITION_PHASE_ARM:
      status = MID_LisanEc_WriteControlWord(pTest->nodeId, MID_LISAN_EC_CONTROL_WORD_ENABLE_OPERATION |
                                                               MID_LISAN_EC_CONTROL_WORD_RELATIVE_POSITION);
      break;
    case APP_POSITION_PHASE_TARGET:
      status = MID_LisanEc_SetTargetPosition(pTest->nodeId, pMove->relativePulses);
      break;
    case APP_POSITION_PHASE_TARGET_READ:
      status = MID_CANopen_Sdo_ReadI32(pTest->nodeId, MID_CANOPEN_OD_TARGET_POSITION, 0U, &pMove->targetReadback);
      if ((status == MID_CANOPEN_OK) && (pMove->targetReadback != pMove->relativePulses))
      {
        APP_PositionTest_Fail(pTest);
      }
      break;
    case APP_POSITION_PHASE_TRIGGER:
      status = MID_LisanEc_WriteControlWord(pTest->nodeId, MID_LISAN_EC_CONTROL_WORD_ENABLE_OPERATION |
                                                               MID_LISAN_EC_CONTROL_WORD_RELATIVE_POSITION |
                                                               MID_LISAN_EC_CONTROL_WORD_NEW_SETPOINT);
      break;
    case APP_POSITION_PHASE_STATUS:
      status = MID_LisanEc_ReadStatusWord(pTest->nodeId, &state);
      if (status == MID_CANOPEN_OK)
      {
        pMove->statusWord = state.raw;
      }
      break;
    case APP_POSITION_PHASE_POSITION:
      status = MID_LisanEc_ReadPosition(pTest->nodeId, &pMove->actualPosition);
      break;
    case APP_POSITION_PHASE_VELOCITY:
      status = MID_LisanEc_ReadVelocity(pTest->nodeId, &pMove->actualVelocity);
      break;
    case APP_POSITION_PHASE_ERROR:
      status = MID_LisanEc_ReadErrorCode(pTest->nodeId, &pMove->errorCode);
      break;
    case APP_POSITION_PHASE_EVALUATE:
      isTransaction = false;
      error = (int64_t) pMove->actualPosition - pMove->expectedPosition;
      if ((MID_LisanEc_ParseState(pMove->statusWord) != MID_LISAN_EC_STATE_OPERATION_ENABLED) ||
          (pMove->errorCode != 0U))
      {
        APP_PositionTest_Fail(pTest);
      }
      else if (((pMove->statusWord & MID_LISAN_EC_STATUS_WORD_TARGET_REACHED) != 0U) &&
               (error >= -CHASSIS_LINK_POSITION_TOLERANCE_PULSES) &&
               (error <= CHASSIS_LINK_POSITION_TOLERANCE_PULSES) &&
               (pMove->actualVelocity >= -CHASSIS_LINK_MOTION_SPEED_TOLERANCE_PPS) &&
               (pMove->actualVelocity <= CHASSIS_LINK_MOTION_SPEED_TOLERANCE_PPS))
      {
        pMove->result = APP_MOTOR_RESULT_VERIFIED;
        pMove->endTick = now;
        pTest->phase = APP_POSITION_PHASE_GAP;
        s_positionNextTick = now;
      }
      else
      {
        pTest->phase = APP_POSITION_PHASE_STATUS;
      }
      break;
    case APP_POSITION_PHASE_GAP:
      isTransaction = false;
      if ((now - s_positionNextTick) >= CHASSIS_LINK_POSITION_GAP_MS)
      {
        if ((pTest->moveIndex + 1U) < APP_POSITION_MOVE_COUNT)
        {
          pTest->moveIndex++;
          APP_PositionTest_BeginMove(pTest);
        }
        else
        {
          pTest->result = APP_MOTOR_RESULT_VERIFIED;
          pTest->phase = APP_POSITION_PHASE_DONE;
          s_positionActive = CANOPEN_MAX_NODE_COUNT;
        }
      }
      break;
    case APP_POSITION_PHASE_HALT:
      if ((now - s_positionNextTick) < CHASSIS_LINK_DRIVE_RETRY_PERIOD_MS)
      {
        isTransaction = false;
        break;
      }
      s_positionNextTick = now;
      pTest->haltAttempts++;
      status = MID_LisanEc_WriteControlWord(pTest->nodeId, MID_LISAN_EC_CONTROL_WORD_HALT);
      if (status == MID_CANOPEN_OK)
      {
        pTest->phase = APP_POSITION_PHASE_HALT_VELOCITY;
      }
      break;
    case APP_POSITION_PHASE_HALT_VELOCITY:
      status = MID_LisanEc_ReadVelocity(pTest->nodeId, &pMove->actualVelocity);
      if ((status == MID_CANOPEN_OK) && (pMove->actualVelocity >= -CHASSIS_LINK_MOTION_SPEED_TOLERANCE_PPS) &&
          (pMove->actualVelocity <= CHASSIS_LINK_MOTION_SPEED_TOLERANCE_PPS))
      {
        pTest->haltConfirmed = true;
        pTest->phase = APP_POSITION_PHASE_DONE;
        s_positionActive = CANOPEN_MAX_NODE_COUNT;
      }
      else
      {
        pTest->phase = APP_POSITION_PHASE_HALT;
      }
      break;
    default:
      isTransaction = false;
      break;
  }
  if (isTransaction)
  {
    pMove->operationStatus[phase] = status;
    pNode = MID_CANopen_GetNode(pTest->nodeId);
    if ((status == MID_CANOPEN_ABORTED) && (pNode != NULL))
    {
      pMove->abortCode[phase] = pNode->lastSdoAbortCode;
    }
    if (phase < APP_POSITION_PHASE_EVALUATE)
    {
      if (status != MID_CANOPEN_OK)
      {
        APP_PositionTest_Fail(pTest);
      }
      else if (pTest->phase == phase)
      {
        pTest->phase = (APP_PositionPhaseTypeDef) (phase + 1U);
      }
    }
  }
  g_motorPositionDebug[index] = *pTest;
#endif
}
