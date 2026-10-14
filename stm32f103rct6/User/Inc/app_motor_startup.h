/**
  ******************************************************************************
  * @file    app_motor_startup.h
  * @brief   电机启动流程与上电自检接口。
  * @author  muhuangan
  * @date    2026-10-13
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
  * 本模块负责把每个轮节点从等心跳、起网络、设模式与轮廓参数到使能逐步推到可下发速度的状态，
  * 每一步都读回对象并核对驱动器状态字，未达标不进入下一步；位置点动、速度点动与双轮一圈自检
  * 由同文件的三个 Process 入口实现，均受 chassis_link_config.h 的开关控制，关闭时不参与编译。
  *
  ******************************************************************************
  */
#ifndef __APP_MOTOR_STARTUP_H
#define __APP_MOTOR_STARTUP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "mid_lisan_ec.h"
#include "chassis_link_config.h"

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
#if (CHASSIS_LINK_MOTOR_DEBUG_ENABLE == 1U)
    APP_MotorCommandDebugTypeDef lastFailure;
    APP_MotorCommandDebugTypeDef firstSteps[APP_MOTOR_STEP_COUNT];
    APP_MotorCommandDebugTypeDef latestSteps[APP_MOTOR_STEP_COUNT];
    APP_MotorCommandDebugTypeDef latest;
    APP_MotorCommandDebugTypeDef firstStart;
    APP_MotorCommandDebugTypeDef firstStop;
#endif
  } APP_MotorStartupDebugTypeDef;

  extern volatile APP_MotorStartupDebugTypeDef g_motorStartupDebug[CANOPEN_MAX_NODE_COUNT];

#if CHASSIS_LINK_POSITION_TEST_ACTIVE
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
#endif

#if (CHASSIS_LINK_SPIN_TEST_ACTIVE == 1)
  /** @defgroup APP_VELOCITY_TEST_Command 速度测试命令顺序
   * @{
   */
  typedef enum
  {
    APP_VELOCITY_COMMAND_FORWARD = 0U,
    APP_VELOCITY_COMMAND_STOP_FORWARD,
    APP_VELOCITY_COMMAND_REVERSE,
    APP_VELOCITY_COMMAND_STOP_REVERSE,
    APP_VELOCITY_COMMAND_COUNT
  } APP_VelocityCommandTypeDef;
  /** @} */
  /** @defgroup APP_VELOCITY_TEST_Phase 速度点动阶段
   * @{
   */
  typedef enum
  {
    APP_VELOCITY_PHASE_QUEUE = 0U,
    APP_VELOCITY_PHASE_WAIT,
    APP_VELOCITY_PHASE_HOLD,
    APP_VELOCITY_PHASE_GAP,
    APP_VELOCITY_PHASE_RETRY_STOP,
    APP_VELOCITY_PHASE_WAIT_STOP,
    APP_VELOCITY_PHASE_DONE
  } APP_VelocityPhaseTypeDef;
  /** @} */
  /** @brief 保留四条速度指令的最终验证结果，首次失败不被停止重试覆盖。 */
  typedef struct
  {
    uint8_t                       nodeId;
    APP_VelocityCommandTypeDef    commandIndex;
    APP_VelocityPhaseTypeDef      phase;
    APP_MotorCommandResultTypeDef result;
    bool                          hasFailure;
    bool                          haltConfirmed;
    uint32_t                      stopRetryCount;
    APP_MotorCommandDebugTypeDef  firstFailure;
    APP_MotorCommandDebugTypeDef  commands[APP_VELOCITY_COMMAND_COUNT];
  } APP_VelocityTestDebugTypeDef;
  extern volatile APP_VelocityTestDebugTypeDef g_motorVelocityDebug[CANOPEN_MAX_NODE_COUNT];
  void APP_MotorStartup_VelocityTestProcess(const uint8_t *pNodeIds, uint8_t nodeCount);
#endif

#if CHASSIS_LINK_SYNC_ROUND_ACTIVE
/** @defgroup APP_SYNC_ROUND_Phase 双轮一圈阶段
 * @{
 */
#define APP_SYNC_ROUND_WHEELS 2U
  typedef enum
  {
    APP_SYNC_PHASE_READY = 0U,
    APP_SYNC_PHASE_INITIAL_VELOCITY,
    APP_SYNC_PHASE_PULSES_PER_REV,
    APP_SYNC_PHASE_START_POSITION,
    APP_SYNC_PHASE_START,
    APP_SYNC_PHASE_TARGET,
    APP_SYNC_PHASE_STATUS,
    APP_SYNC_PHASE_POSITION,
    APP_SYNC_PHASE_VELOCITY,
    APP_SYNC_PHASE_ERROR,
    APP_SYNC_PHASE_CONTROL,
    APP_SYNC_PHASE_WRITE_SPEED,
    APP_SYNC_PHASE_STOP_SEND,
    APP_SYNC_PHASE_STOP_TARGET,
    APP_SYNC_PHASE_STOP_VELOCITY,
    APP_SYNC_PHASE_STOP_WAIT,
    APP_SYNC_PHASE_DONE,
    APP_SYNC_PHASE_COUNT
  } APP_SyncRoundPhaseTypeDef;
  typedef enum
  {
    APP_SYNC_FAILURE_NONE = 0U,
    APP_SYNC_FAILURE_READY_TIMEOUT,
    APP_SYNC_FAILURE_GEOMETRY,
    APP_SYNC_FAILURE_SDO,
    APP_SYNC_FAILURE_DRIVE,
    APP_SYNC_FAILURE_POSITION,
    APP_SYNC_FAILURE_STALL,
    APP_SYNC_FAILURE_TIMEOUT
  } APP_SyncRoundFailureTypeDef;
  /** @} */
  /** @brief 单轮的每圈参数、累计位置和最新有效反馈，读取状态必须为 OK 才可使用数值。 */
  typedef struct
  {
    uint8_t                  nodeId;
    uint32_t                 pulsesPerMotorRev;
    uint32_t                 targetPulses;
    int32_t                  startPosition;
    int32_t                  actualPosition;
    int32_t                  travelledPulses;
    int32_t                  remainingPulses;
    int32_t                  commandSpeed;
    int32_t                  actualVelocity;
    int32_t                  targetReadback;
    int32_t                  peakVelocity;
    uint16_t                 statusWord;
    uint16_t                 errorCode;
    uint32_t                 startCommandTick;
    uint32_t                 lastProgressTick;
    uint32_t                 lastAbortCode;
    bool                     done;
    bool                     haltConfirmed;
    MID_CANopenStatusTypeDef operationStatus[APP_SYNC_PHASE_COUNT];
  } APP_SyncWheelDebugTypeDef;
  /** @brief 双轮一起运行一次，任一失败停止两轮并保留失败当时的单轮快照。 */
  typedef struct
  {
    APP_MotorCommandResultTypeDef result;
    APP_SyncRoundPhaseTypeDef     phase;
    uint8_t                       wheelIndex;
    bool                          started;
    bool                          haltConfirmed;
    uint32_t                      startTick;
    uint32_t                      runTick;
    uint32_t                      timeoutMs;
    uint32_t                      stopAttempts;
    APP_SyncRoundFailureTypeDef   failureReason;
    APP_SyncRoundPhaseTypeDef     failedPhase;
    MID_CANopenStatusTypeDef      failureStatus;
    APP_SyncWheelDebugTypeDef     firstFailure;
    APP_SyncWheelDebugTypeDef     wheels[APP_SYNC_ROUND_WHEELS];
  } APP_SyncRoundDebugTypeDef;
  extern volatile APP_SyncRoundDebugTypeDef g_motorSyncDebug;
  void APP_MotorStartup_SyncRoundProcess(const uint8_t *pNodeIds, uint8_t nodeCount);
#endif

  MID_CANopenStatusTypeDef      APP_MotorStartup_AddNode(uint8_t nodeId);
  void                          APP_MotorStartup_Process(void);
  bool                          APP_MotorStartup_IsReady(uint8_t nodeId);
#if (CHASSIS_LINK_SPIN_TEST_ACTIVE == 1) || (CHASSIS_LINK_SYNC_ROUND_ACTIVE == 1)
  MID_CANopenStatusTypeDef      APP_MotorStartup_RequestVelocity(uint8_t nodeId, int32_t velocity);
#endif
  APP_MotorCommandResultTypeDef APP_MotorStartup_GetMotionResult(uint8_t nodeId);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MOTOR_STARTUP_H */
