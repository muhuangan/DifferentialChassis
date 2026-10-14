/**
 ******************************************************************************
 * @file    app_motor_startup.h
 * @brief   电机启动命令与读回验证记录。
 * @author  muhuangan
 * @date    2026-10-13
 * @version V1.0.0
 ******************************************************************************
 * @attention
 * Copyright 2026 muhuangan.
 * This software is licensed under the MIT License in the project LICENSE file.
 ******************************************************************************
 */
#ifndef __APP_MOTOR_STARTUP_H
#define __APP_MOTOR_STARTUP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "mid_lisan_ec.h"

  /** @defgroup APP_MOTOR_STARTUP_Step 电机启动步骤
   * @{
   */
  typedef enum
  {
    APP_MOTOR_STEP_WAIT_HEARTBEAT = 0U,
    APP_MOTOR_STEP_NMT_START = 1U,
    APP_MOTOR_STEP_MODE = 2U,
    APP_MOTOR_STEP_ACCELERATION = 3U,
    APP_MOTOR_STEP_DECELERATION = 4U,
    APP_MOTOR_STEP_ZERO_VELOCITY = 5U,
    APP_MOTOR_STEP_SHUTDOWN = 6U,
    APP_MOTOR_STEP_SWITCH_ON = 7U,
    APP_MOTOR_STEP_ENABLE_OPERATION = 8U,
    APP_MOTOR_STEP_PROFILE_SPEED = 9U,
    APP_MOTOR_STEP_COUNT = 10U,
    APP_MOTOR_STEP_MOTION = 10U
  } APP_MotorStartupStepTypeDef;
  /** @} */

  /** @defgroup APP_MOTOR_STARTUP_Result 命令验证结果
   * @{
   */
  typedef enum
  {
    APP_MOTOR_RESULT_NOT_RUN = 0U,
    APP_MOTOR_RESULT_PENDING = 1U,
    APP_MOTOR_RESULT_VERIFIED = 2U,
    APP_MOTOR_RESULT_FAILED = 3U,
    APP_MOTOR_RESULT_SKIPPED = 4U
  } APP_MotorCommandResultTypeDef;
  /** @} */

  /** @brief 单条命令的独立应答与读回记录，只有对应读取返回 OK 时数据才有效。 */
  typedef struct
  {
    APP_MotorStartupStepTypeDef   step;
    APP_MotorCommandResultTypeDef result;
    uint32_t                      sequence;
    uint32_t                      startTick;
    uint32_t                      endTick;
    uint16_t                      objectIndex;
    int32_t                       commandValue;
    int32_t                       readbackValue;
    MID_CANopenStatusTypeDef      commandStatus;
    MID_CANopenStatusTypeDef      statusReadStatus;
    MID_CANopenStatusTypeDef      verifyReadStatus;
    MID_CANopenStatusTypeDef      velocityReadStatus;
    MID_CANopenStatusTypeDef      errorReadStatus;
    uint32_t                      commandAbortCode;
    uint32_t                      readAbortCode;
    uint32_t                      heartbeatCount;
    MID_CANopenNmtStateTypeDef    nmtState;
    uint16_t                      statusWord;
    MID_LisanEcStateTypeDef       driveState;
    int32_t                       actualVelocity;
    uint16_t                      errorCode;
    bool                          commandAccepted;
    bool                          effectConfirmed;
  } APP_MotorCommandDebugTypeDef;

  /** @brief 每节点的首次启动历史和最近命令，首次历史在重试中保持不变。 */
  typedef struct
  {
    uint8_t                      nodeId;
    bool                         isRegistered;
    bool                         isReady;
    bool                         firstAttemptFinished;
    uint32_t                     attemptCount;
    APP_MotorStartupStepTypeDef  currentStep;
    APP_MotorStartupStepTypeDef  failedStep;
    uint32_t                     failureCount;
    APP_MotorCommandDebugTypeDef lastFailure;
    APP_MotorCommandDebugTypeDef firstSteps[APP_MOTOR_STEP_COUNT];
    APP_MotorCommandDebugTypeDef latestSteps[APP_MOTOR_STEP_COUNT];
    APP_MotorCommandDebugTypeDef latest;
    APP_MotorCommandDebugTypeDef firstStart;
    APP_MotorCommandDebugTypeDef firstStop;
  } APP_MotorStartupDebugTypeDef;

  extern volatile APP_MotorStartupDebugTypeDef g_motorStartupDebug[CANOPEN_MAX_NODE_COUNT];

/** @defgroup APP_POSITION_TEST_Phase 位置点动阶段
 * @{
 */
#define APP_POSITION_MOVE_COUNT 2U
  typedef enum
  {
    APP_POSITION_PHASE_MODE = 0U,
    APP_POSITION_PHASE_BASELINE,
    APP_POSITION_PHASE_ARM,
    APP_POSITION_PHASE_TARGET,
    APP_POSITION_PHASE_TARGET_READ,
    APP_POSITION_PHASE_TRIGGER,
    APP_POSITION_PHASE_STATUS,
    APP_POSITION_PHASE_POSITION,
    APP_POSITION_PHASE_VELOCITY,
    APP_POSITION_PHASE_ERROR,
    APP_POSITION_PHASE_EVALUATE,
    APP_POSITION_PHASE_GAP,
    APP_POSITION_PHASE_HALT,
    APP_POSITION_PHASE_HALT_VELOCITY,
    APP_POSITION_PHASE_DONE,
    APP_POSITION_PHASE_COUNT
  } APP_PositionPhaseTypeDef;
  /** @} */

  /** @brief 一段相对运动的命令应答和实际到位结果，操作状态为 OK 才可使用对应数据。 */
  typedef struct
  {
    APP_MotorCommandResultTypeDef result;
    APP_PositionPhaseTypeDef      failedPhase;
    MID_CANopenStatusTypeDef      operationStatus[APP_POSITION_PHASE_COUNT];
    uint32_t                      abortCode[APP_POSITION_PHASE_COUNT];
    uint32_t                      startTick;
    uint32_t                      endTick;
    int32_t                       relativePulses;
    int32_t                       startPosition;
    int32_t                       expectedPosition;
    int32_t                       actualPosition;
    int32_t                       targetReadback;
    int32_t                       actualVelocity;
    MID_LisanEcModeTypeDef        actualMode;
    uint16_t                      statusWord;
    uint16_t                      errorCode;
  } APP_PositionMoveDebugTypeDef;

  /** @brief 每个节点只执行一次正反点动，任一步失败后只尝试停止，不重发位移触发。 */
  typedef struct
  {
    uint8_t                       nodeId;
    uint8_t                       moveIndex;
    APP_PositionPhaseTypeDef      phase;
    APP_MotorCommandResultTypeDef result;
    bool                          haltConfirmed;
    uint32_t                      haltAttempts;
    APP_PositionMoveDebugTypeDef  moves[APP_POSITION_MOVE_COUNT];
  } APP_PositionTestDebugTypeDef;

  extern volatile APP_PositionTestDebugTypeDef g_motorPositionDebug[CANOPEN_MAX_NODE_COUNT];
  void                                         APP_MotorStartup_PositionTestProcess(void);

  MID_CANopenStatusTypeDef      APP_MotorStartup_AddNode(uint8_t nodeId);
  void                          APP_MotorStartup_Process(void);
  bool                          APP_MotorStartup_IsReady(uint8_t nodeId);
  MID_CANopenStatusTypeDef      APP_MotorStartup_RequestVelocity(uint8_t nodeId, int32_t velocity);
  APP_MotorCommandResultTypeDef APP_MotorStartup_GetMotionResult(uint8_t nodeId);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MOTOR_STARTUP_H */
