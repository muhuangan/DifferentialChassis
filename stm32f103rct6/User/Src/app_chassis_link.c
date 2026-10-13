/**
  ******************************************************************************
  * @file    app_chassis_link.c
  * @brief   底盘上位机链路应用层实现。
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

#include "app_chassis_link.h"

#include <stddef.h>

#include "cmsis_os.h"
#include "can.h"
#include "chassis_link_config.h"
#include "mid_uart_ring.h"
#include "mid_chassis_protocol.h"
#include "mid_canopen.h"
#include "mid_lisan_ec.h"

/**
  * @defgroup APP_CHASSIS_LINK_Semaphore_Config 链路信号量配置
  * @{
  */
#define APP_CHASSIS_LINK_SEMAPHORE_MAX_COUNT   1U /**< 二值信号量上限，重复释放不会累积。 */
#define APP_CHASSIS_LINK_SEMAPHORE_INIT_COUNT  0U /**< 初始计数为 0，任务启动后先阻塞等待。 */
/**
  * @}
  */

static StaticSemaphore_t s_rxSemaphoreBuffer;    /**< 接收信号量的静态控制块。 */
static osSemaphoreId_t   s_hRxSemaphore = NULL;  /**< 串口数据到达信号量，中断侧释放、任务侧等待。 */

/* 信号量属性，绑定静态控制块，避免动态分配。 */
static const osSemaphoreAttr_t s_rxSemaphoreAttr =
{
  .name      = "chassisRx",
  .attr_bits = 0U,
  .cb_mem    = &s_rxSemaphoreBuffer,
  .cb_size   = sizeof(s_rxSemaphoreBuffer)
};

static uint8_t s_parseBuffer[CHASSIS_LINK_PARSE_CHUNK_BYTES];   /**< 从环形缓冲区批量取数的中转缓冲区。 */

/* 左右轮节点地址组，顺序由 CHASSIS_LINK_WHEEL_NODE_ID_LIST 决定，取用下标统一用 CHASSIS_LINK_WHEEL_INDEX_* 表达。 */
static const uint8_t s_wheelNodeIds[CHASSIS_LINK_WHEEL_COUNT] = CHASSIS_LINK_WHEEL_NODE_ID_LIST;

static int32_t  s_targetLeftSpeed = 0;           /**< 上位机最新下发的左轮目标速度，单位脉冲每秒。 */
static int32_t  s_targetRightSpeed = 0;          /**< 上位机最新下发的右轮目标速度，单位脉冲每秒。 */
static int32_t  s_sentLeftSpeed = 0;             /**< 最近一次实际下发的左轮速度，单位脉冲每秒。 */
static int32_t  s_sentRightSpeed = 0;            /**< 最近一次实际下发的右轮速度，单位脉冲每秒。 */
static uint32_t s_lastValidFrameTick = 0U;       /**< 最近一次收到合法帧的时间戳，单位毫秒。 */
static uint32_t s_lastDispatchTick = 0U;         /**< 最近一次下发速度的时间戳，单位毫秒。 */
static uint32_t s_lastDriveInitTick = 0U;        /**< 最近一次尝试初始化驱动器的时间戳，单位毫秒。 */
static bool     s_hasValidFrame = false;         /**< 是否收到过至少一帧合法数据。 */
static bool     s_isCommunicationLost = false;   /**< 上位机通信超时标志。 */
static bool     s_isDriveReady = false;          /**< 两轮驱动器是否已完成模式设置与使能。 */

static volatile uint32_t s_validFrameCount = 0U;         /**< 合法帧累计数。 */
static volatile uint32_t s_headerMismatchCount = 0U;     /**< 帧头失配累计数。 */
static volatile uint32_t s_tailMismatchCount = 0U;       /**< 帧尾失配累计数。 */
static volatile uint32_t s_rangeErrorCount = 0U;         /**< 范围校验失败累计数。 */
static volatile uint32_t s_dispatchCount = 0U;           /**< 单轮转速下发成功累计数。 */
static volatile uint32_t s_dispatchFailureCount = 0U;    /**< 转速下发失败累计数。 */
static volatile uint32_t s_lastSdoAbortCode = 0U;        /**< 最近一次下发失败对应节点的 SDO 中止码。 */
static volatile uint32_t s_commLostCount = 0U;           /**< 通信超时触发停机的累计次数。 */

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 1U)
/* 自检节点地址组，顺序由 CHASSIS_LINK_SELF_TEST_NODE_ID_LIST 决定，点动时按下标依次轮转。 */
static const uint8_t s_selfTestNodeIds[CHASSIS_LINK_SELF_TEST_NODE_COUNT] = CHASSIS_LINK_SELF_TEST_NODE_ID_LIST;
static bool     s_isSelfTestNodeReady[CHASSIS_LINK_SELF_TEST_NODE_COUNT]; /**< 各节点是否已完成配置与使能。 */
static uint32_t s_selfTestInitTick[CHASSIS_LINK_SELF_TEST_NODE_COUNT];    /**< 各节点最近一次尝试配置的时间戳，0 表示尚未尝试。 */
static bool     s_isSelfTestRunning = false;     /**< 自检相位，true 为当前节点转动、false 为节点间停顿。 */
static bool     s_isSelfTestStarted = false;     /**< 是否已下发过自检首个速度，用于保证上电即转。 */
static uint8_t  s_selfTestActiveIndex = 0U;      /**< 当前点动节点在自检地址组中的下标。 */
static uint32_t s_selfTestPhaseTick = 0U;        /**< 最近一次自检相位翻转的时间戳，单位毫秒。 */
#endif

static MID_CANopenStatusTypeDef APP_ChassisLink_ConfigureNode(uint8_t nodeId);
static MID_CANopenStatusTypeDef APP_ChassisLink_ConfigureDrive(void);
static void APP_ChassisLink_EnsureDriveReady(void);
static void APP_ChassisLink_DrainAndParse(void);
static void APP_ChassisLink_OnValidFrame(const MID_ChassisPayloadTypeDef *pPayload);
static void APP_ChassisLink_Dispatch(void);
static void APP_ChassisLink_SendSpeeds(int32_t leftSpeed, int32_t rightSpeed);
static void APP_ChassisLink_SendOneWheel(uint8_t wheelIndex, int32_t speed);
static void APP_ChassisLink_RecordFailure(uint8_t nodeId);
static int32_t APP_ChassisLink_ApplyInvert(int32_t speed, uint32_t invertFlag);
static bool APP_ChassisLink_IsSpeedChanged(int32_t target, int32_t sent);
static bool APP_ChassisLink_IsCommunicationLost(uint32_t currentTick);

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 1U)
static void APP_ChassisLink_EnsureSelfTestNodesReady(void);
static void APP_ChassisLink_RunSelfTest(void);
#endif

/**
  * @brief  把单个轮节点配置为轮廓速度模式并使其使能。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX。
  * @retval MID_CANOPEN_OK 配置成功。
  * @retval 其他           首个失败步骤的 SDO 错误码。
  * @note   执行顺序严格对齐手册 4.2.2 轮廓速度模式示例（p37~p38）：
  *         1) 先发 NMT 进入运行（Start Remote Node）。手册 4.1.3 状态字 bit9 说明
  *            「在 ProOP 以下时 bit9 为 0，此时控制字 6040h 的命令将无法执行」，
  *            因此必须先确保通信状态机在 Operational；该指令无应答、重复发送幂等，安全。
  *         2) 写工作模式 0x6060=3（手册 p30 入口表，PV 模式）。
  *         3) 写梯形加速度 0x6083 与减速度 0x6084。手册 PV 模式说明原文：
  *            「一旦设置了速度、加速度和减速度，驱动器将根据加速度值驱动电机加速到运行速度」，
  *            不写就只能沿用驱动器内部旧值，加减速过小时表现为「使能了但起不来」。
  *         4) 先把目标速度 0x60FF 清 0，再下发控制字序列。
  *            手册示例的顺序是 0x60FF 在 0x07/0x0F 之前设置；若顺序颠倒，
  *            使能瞬间驱动器会按 0x60FF 中残留的旧值先运行一拍，才被后续目标速度覆盖。
  *         5) 下发 0x06 → 0x07 → 0x0F，对应手册 4.1.1 状态切换 2/3/4（p25）。
  *         任一步失败立即返回，不再执行后续步骤。
  */
static MID_CANopenStatusTypeDef APP_ChassisLink_ConfigureNode(uint8_t nodeId)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;

  /* 1) 确保通信状态机在 Operational，否则控制字命令不执行（手册 4.1.3 bit9）。 */
  (void)MID_CANopen_Nmt_Control(nodeId, MID_CANOPEN_NMT_START_REMOTE_NODE);

  /* 2) 工作模式：轮廓速度模式。 */
  status = MID_LisanEc_SetOperationMode(nodeId, MID_LISAN_EC_MODE_PROFILE_VELOCITY);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

#if (CHASSIS_LINK_ENABLE_PROFILE_RAMP == 1U)
  /* 3) 轮廓加减速度，手册 4.2.2 明确要求设置。 */
  status = MID_LisanEc_SetProfileAcceleration(nodeId, CHASSIS_LINK_PROFILE_ACCELERATION_PPS2);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  status = MID_LisanEc_SetProfileDeceleration(nodeId, CHASSIS_LINK_PROFILE_DECELERATION_PPS2);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }
#endif

  /* 4) 先把目标速度清 0，避免使能瞬间按残留旧值运行。 */
  status = MID_LisanEc_SetTargetVelocity(nodeId, 0);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  /* 5) 控制字序列 0x06 → 0x07 → 0x0F，对应手册 4.1.1 状态切换 2/3/4。 */
  return MID_LisanEc_EnableDrive(nodeId);
}

/**
  * @brief  逐一配置全部轮节点。
  * @param  None
  * @retval MID_CANOPEN_OK 全部节点配置成功。
  * @retval 其他           遇到的首个失败节点的 SDO 错误码。
  * @note   按 s_wheelNodeIds 的顺序遍历，首个失败即中止，已配置成功的节点保持原状态。
  */
static MID_CANopenStatusTypeDef APP_ChassisLink_ConfigureDrive(void)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;
  uint8_t index = 0U;

  for (index = 0U; index < CHASSIS_LINK_WHEEL_COUNT; index++)
  {
    status = APP_ChassisLink_ConfigureNode(s_wheelNodeIds[index]);
    if (status != MID_CANOPEN_OK)
    {
      return status;
    }
  }

  return status;
}

/**
  * @brief  按安装方向把速度统一到整车正方向。
  * @param  speed       上位机下发的原始速度，单位脉冲每秒。
  * @param  invertFlag  非 0 表示该侧电机安装方向相反，需要对调符号。
  * @retval 转成整车正方向后的速度，单位脉冲每秒。
  * @note   仅在方向标志非 0 时返回相反数，用于消除左右电机对装造成的方向差异。
  */
static int32_t APP_ChassisLink_ApplyInvert(int32_t speed, uint32_t invertFlag)
{
  return ((invertFlag != 0U) ? (-speed) : speed);
}

/**
  * @brief  判断目标速度相对已下发速度的变化量是否足够大。
  * @param  target  目标速度，单位脉冲每秒。
  * @param  sent    最近一次下发的速度，单位脉冲每秒。
  * @retval true   绝对差值大于 CHASSIS_LINK_SPEED_DEADBAND_PPS，需要重新下发。
  * @retval false  差值落在死区内，视为无变化。
  * @note   死区用于滤除目标值的细微抖动，避免持续产生无意义的 CAN 报文。
  */
static bool APP_ChassisLink_IsSpeedChanged(int32_t target, int32_t sent)
{
  int32_t delta = (target >= sent) ? (target - sent) : (sent - target);

  return (delta > CHASSIS_LINK_SPEED_DEADBAND_PPS);
}

/**
  * @brief  判断上位机数据是否已超时。
  * @param  currentTick  当前时间戳，单位毫秒。
  * @retval true   距最近一次合法帧已超过 CHASSIS_LINK_COMM_TIMEOUT_MS。
  * @retval false  通信正常，或 CHASSIS_LINK_ENABLE_COMM_TIMEOUT_STOP 关闭了超时停机。
  * @note   超时门限为 CHASSIS_LINK_COMM_TIMEOUT_MS；编译期关闭超时停机开关时恒返回 false，不产生运行时开销。
  */
static bool APP_ChassisLink_IsCommunicationLost(uint32_t currentTick)
{
#if (CHASSIS_LINK_ENABLE_COMM_TIMEOUT_STOP == 1U)
  return ((currentTick - s_lastValidFrameTick) > CHASSIS_LINK_COMM_TIMEOUT_MS);
#else
  (void)currentTick;

  return false;
#endif
}

/**
  * @brief  记录一次转速下发失败。
  * @param  nodeId  下发失败的节点地址。
  * @retval None
  * @note   失败计数累加，并把该节点最近一次的 SDO 中止码拷入 s_lastSdoAbortCode，
  *         便于通过统计快照定位失败原因。
  */
static void APP_ChassisLink_RecordFailure(uint8_t nodeId)
{
  MID_CANopenNodeTypeDef *pNode = MID_CANopen_GetNode(nodeId);

  s_dispatchFailureCount++;

  if (pNode != NULL)
  {
    s_lastSdoAbortCode = pNode->lastSdoAbortCode;
  }
}

/**
  * @brief  向单个轮节点写入目标速度对象 0x60FF。
  * @param  wheelIndex  轮下标，取 CHASSIS_LINK_WHEEL_INDEX_LEFT 或 CHASSIS_LINK_WHEEL_INDEX_RIGHT。
  * @param  speed       目标速度，单位脉冲每秒，符号表示方向。
  * @retval None
  * @note   写入成功累加 s_dispatchCount，失败转入 APP_ChassisLink_RecordFailure 记录原因。
  */
static void APP_ChassisLink_SendOneWheel(uint8_t wheelIndex, int32_t speed)
{
  uint8_t nodeId = s_wheelNodeIds[wheelIndex];

  if (MID_LisanEc_SetTargetVelocity(nodeId, speed) == MID_CANOPEN_OK)
  {
    s_dispatchCount++;
  }
  else
  {
    APP_ChassisLink_RecordFailure(nodeId);
  }
}

/**
  * @brief  一次下发左右两轮的目标速度。
  * @param  leftSpeed   左轮目标速度，单位脉冲每秒。
  * @param  rightSpeed  右轮目标速度，单位脉冲每秒。
  * @retval None
  * @note   先刷新下发时间戳，再分别下发两轮；无论单轮成功与否都把本次目标值记为已下发，
  *         避免下一轮立即重试形成重试风暴，补发由强制刷新周期承担。
  */
static void APP_ChassisLink_SendSpeeds(int32_t leftSpeed, int32_t rightSpeed)
{
  s_lastDispatchTick = HAL_GetTick();

  APP_ChassisLink_SendOneWheel(CHASSIS_LINK_WHEEL_INDEX_LEFT, leftSpeed);
  APP_ChassisLink_SendOneWheel(CHASSIS_LINK_WHEEL_INDEX_RIGHT, rightSpeed);

  /* 无论成功与否都记录本次目标值，避免下一拍立刻重试形成重试风暴，补发由强制刷新周期承担。 */
  s_sentLeftSpeed = leftSpeed;
  s_sentRightSpeed = rightSpeed;
}

/**
  * @brief  用一帧合法数据刷新目标速度与接收时刻。
  * @param  pPayload  已解析出的负载，不可为 NULL。
  * @retval None
  * @note   左右轮速度先按 CHASSIS_LINK_SPEED_INVERT_LEFT 与 CHASSIS_LINK_SPEED_INVERT_RIGHT
  *         统一方向，再写入目标值；同时累加合法帧计数并刷新最近接收时刻，供失联判定使用。
  */
static void APP_ChassisLink_OnValidFrame(const MID_ChassisPayloadTypeDef *pPayload)
{
  s_validFrameCount++;
  s_targetLeftSpeed = APP_ChassisLink_ApplyInvert(pPayload->leftSpeed, CHASSIS_LINK_SPEED_INVERT_LEFT);
  s_targetRightSpeed = APP_ChassisLink_ApplyInvert(pPayload->rightSpeed, CHASSIS_LINK_SPEED_INVERT_RIGHT);
  s_lastValidFrameTick = HAL_GetTick();
  s_hasValidFrame = true;
}

/**
  * @brief  取走环形缓冲区中已到达的数据并逐字节解析成帧。
  * @param  None
  * @retval None
  * @note   每轮从环形缓冲区批量取 CHASSIS_LINK_PARSE_CHUNK_BYTES 字节到中转缓冲再逐字节投喂解析器，
  *         本轮总取数不超过一环容量，防止数据持续到达时长时间占住任务；
  *         解析结果按合法帧、帧头失配、帧尾失配、范围错误分别累加统计。
  */
static void APP_ChassisLink_DrainAndParse(void)
{
  MID_ChassisPayloadTypeDef payload;
  uint16_t available = MID_UartRing_GetAvailable();
  uint16_t budget = MID_UART_RING_SIZE_BYTES;
  uint16_t count = 0U;
  uint16_t index = 0U;

  while ((available > 0U) && (budget > 0U))
  {
    count = (available < CHASSIS_LINK_PARSE_CHUNK_BYTES) ? available : (uint16_t)CHASSIS_LINK_PARSE_CHUNK_BYTES;
    count = MID_UartRing_Read(s_parseBuffer, count);

    if (count == 0U)
    {
      break;
    }

    for (index = 0U; index < count; index++)
    {
      switch (MID_ChassisProtocol_ParseByte(s_parseBuffer[index], &payload))
      {
        case MID_CHASSIS_PARSE_VALID:
          APP_ChassisLink_OnValidFrame(&payload);
          break;
        case MID_CHASSIS_PARSE_HEADER_MISMATCH:
          s_headerMismatchCount++;
          break;
        case MID_CHASSIS_PARSE_TAIL_MISMATCH:
          s_tailMismatchCount++;
          break;
        case MID_CHASSIS_PARSE_RANGE_ERROR:
          s_rangeErrorCount++;
          break;
        default:
          break;
      }
    }

    budget = (uint16_t)(budget - count);
    available = MID_UartRing_GetAvailable();
  }
}

/**
  * @brief  在驱动器未就绪时按周期重试配置与使能。
  * @param  None
  * @retval None
  * @note   已就绪则直接返回；未就绪但距上次尝试不足 CHASSIS_LINK_DRIVE_RETRY_PERIOD_MS 也直接返回。
  *         每次尝试都刷新重试时间戳，并把配置结果写回 s_isDriveReady。
  */
static void APP_ChassisLink_EnsureDriveReady(void)
{
  uint32_t currentTick = HAL_GetTick();

  if (s_isDriveReady != false)
  {
    return;
  }

  if ((currentTick - s_lastDriveInitTick) < CHASSIS_LINK_DRIVE_RETRY_PERIOD_MS)
  {
    return;
  }

  s_lastDriveInitTick = currentTick;
  s_isDriveReady = (APP_ChassisLink_ConfigureDrive() == MID_CANOPEN_OK) ? true : false;
}

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 1U)
/**
  * @brief  按重试周期让全部自检节点完成配置与使能。
  * @param  None
  * @retval None
  * @note   逐个遍历 CHASSIS_LINK_SELF_TEST_NODE_ID_LIST：已就绪的跳过，未就绪的按
  *         CHASSIS_LINK_DRIVE_RETRY_PERIOD_MS 节流后调用 APP_ChassisLink_ConfigureNode，
  *         时间戳为 0 视为尚未尝试过，因此首次进入立即尝试。
  *         不复用要求两轮全就绪的 s_isDriveReady，且单个节点失败不影响其余节点，
  *         因此在只接部分驱动器、或个别节点地址设错的现场，其余节点仍能点动起来。
  * @warning 节点不存在时每条 SDO 都会等满 CANOPEN_SDO_TIMEOUT_MS，本函数在链路任务上下文中串行阻塞；
  *          全部自检节点均不在线时单次调用最坏阻塞约为「节点数 × 每节点 SDO 条数 × 超时上限」，
  *          属自检专用路径，开关置 0 后不产生任何开销。
  */
static void APP_ChassisLink_EnsureSelfTestNodesReady(void)
{
  uint32_t currentTick = HAL_GetTick();
  uint8_t index = 0U;

  for (index = 0U; index < CHASSIS_LINK_SELF_TEST_NODE_COUNT; index++)
  {
    if (s_isSelfTestNodeReady[index] != false)
    {
      continue;
    }

    if ((s_selfTestInitTick[index] != 0U) &&
        ((currentTick - s_selfTestInitTick[index]) < CHASSIS_LINK_DRIVE_RETRY_PERIOD_MS))
    {
      continue;
    }

    s_selfTestInitTick[index] = currentTick;
    s_isSelfTestNodeReady[index] =
      (APP_ChassisLink_ConfigureNode(s_selfTestNodeIds[index]) == MID_CANOPEN_OK) ? true : false;
  }
}

/**
  * @brief  依次点动自检节点，一次上电即可看清每个节点地址对应哪个轮子。
  * @param  None
  * @retval None
  * @note   同一时刻只有一个节点转动：首次执行时把下标 0 的节点置为转动相位并立即下发
  *         CHASSIS_LINK_SELF_TEST_SPEED_PPS，保证上电即可看到轮子动；转动满 CHASSIS_LINK_SELF_TEST_ON_MS
  *         后写 0 进入停顿，停满 CHASSIS_LINK_SELF_TEST_OFF_MS 后下标加一，整个地址组走完从头循环。
  *         停止相位只写 0 而不断使能（保持锁轴，不反复脱机）；相间判定用无符号差值，天然兼容 HAL_GetTick 回绕。
  *         若当前下标的节点尚未就绪，只把下标前移一拍并重置相位，避免个别节点配置失败卡死整个自检。
  *         写入失败时转入 APP_ChassisLink_RecordFailure 记录，便于通过统计快照观察。
  * @warning 自检与上位机链路共用同一目标速度：若上位机同时发帧，APP_ChassisLink_Dispatch 会在每轮循环覆盖自检速度，
  *          因此自检期间不要向上位机发送速度帧。
  */
static void APP_ChassisLink_RunSelfTest(void)
{
  uint32_t currentTick = HAL_GetTick();
  uint32_t phaseMs = 0U;
  int32_t speed = 0;

  if (s_isSelfTestNodeReady[s_selfTestActiveIndex] == false)
  {
    /* 当前节点还没配置成功，先轮到下一个，避免被单个异常节点卡住。 */
    s_selfTestActiveIndex = (uint8_t)((s_selfTestActiveIndex + 1U) % CHASSIS_LINK_SELF_TEST_NODE_COUNT);
    s_isSelfTestStarted = false;

    return;
  }

  if (s_isSelfTestStarted == false)
  {
    s_isSelfTestStarted = true;
    s_isSelfTestRunning = true;
    s_selfTestPhaseTick = currentTick;
  }
  else
  {
    phaseMs = (s_isSelfTestRunning != false) ? CHASSIS_LINK_SELF_TEST_ON_MS : CHASSIS_LINK_SELF_TEST_OFF_MS;

    if ((currentTick - s_selfTestPhaseTick) < phaseMs)
    {
      return;
    }

    s_selfTestPhaseTick = currentTick;

    if (s_isSelfTestRunning != false)
    {
      s_isSelfTestRunning = false;
    }
    else
    {
      s_selfTestActiveIndex = (uint8_t)((s_selfTestActiveIndex + 1U) % CHASSIS_LINK_SELF_TEST_NODE_COUNT);
      s_isSelfTestRunning = true;
    }
  }

  speed = (s_isSelfTestRunning != false) ? CHASSIS_LINK_SELF_TEST_SPEED_PPS : 0;

  if (MID_LisanEc_SetTargetVelocity(s_selfTestNodeIds[s_selfTestActiveIndex], speed) != MID_CANOPEN_OK)
  {
    APP_ChassisLink_RecordFailure(s_selfTestNodeIds[s_selfTestActiveIndex]);
  }
}
#endif

/**
  * @brief  判定本次是否需要下发转速，需要则下发。
  * @param  None
  * @retval None
  * @note   触发条件有三：首次判定上位机失联时把目标速度清零并立即下发一次实现超时停机；
  *         目标速度相对已下发速度超出死区；距上次下发达到 CHASSIS_LINK_REFRESH_PERIOD_MS。
  *         尚未收到过合法帧时直接返回，不向驱动器下发任何指令。
  */
static void APP_ChassisLink_Dispatch(void)
{
  uint32_t currentTick = HAL_GetTick();
  bool needDispatch = false;

  if (s_hasValidFrame == false)
  {
    return;
  }

  if (APP_ChassisLink_IsCommunicationLost(currentTick) != false)
  {
    if (s_isCommunicationLost == false)
    {
      s_isCommunicationLost = true;
      s_targetLeftSpeed = 0;
      s_targetRightSpeed = 0;
      s_commLostCount++;
      needDispatch = true;
    }
  }
  else
  {
    s_isCommunicationLost = false;
  }

  if ((needDispatch == false) && (s_isCommunicationLost == false))
  {
    if ((APP_ChassisLink_IsSpeedChanged(s_targetLeftSpeed, s_sentLeftSpeed) != false) ||
        (APP_ChassisLink_IsSpeedChanged(s_targetRightSpeed, s_sentRightSpeed) != false))
    {
      needDispatch = true;
    }
  }

  if ((needDispatch == false) && ((currentTick - s_lastDispatchTick) >= CHASSIS_LINK_REFRESH_PERIOD_MS))
  {
    needDispatch = true;
  }

  if (needDispatch != false)
  {
    APP_ChassisLink_SendSpeeds(s_targetLeftSpeed, s_targetRightSpeed);
  }
}

/**
  * @brief  初始化底盘链路，须在调度器启动前调用。
  * @param  None
  * @retval MID_CANOPEN_OK            初始化成功。
  * @retval MID_CANOPEN_INVALID_PARAM CAN 句柄非法。
  * @retval MID_CANOPEN_ERROR         协议栈、节点注册、信号量或串口接收启动失败。
  * @note   执行顺序：初始化 CANopen 协议栈；注册全部轮节点，并在自检开关打开时补注册自检节点；
  *         创建绑定静态控制块的二值信号量；
  *         配置并使能两轮驱动器，此步失败不中止启动而交由任务周期重试；
  *         注册串口通知回调并以循环 DMA 加空闲中断启动 UART4 接收。
  */
MID_CANopenStatusTypeDef APP_ChassisLink_Init(void)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;
  uint8_t index = 0U;

  status = MID_CANopen_Init(&hcan);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  for (index = 0U; index < CHASSIS_LINK_WHEEL_COUNT; index++)
  {
    status = MID_CANopen_AddNode(s_wheelNodeIds[index]);
    if (status != MID_CANOPEN_OK)
    {
      return status;
    }
  }

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 1U)
  /* 自检节点必须注册后才能收发 SDO，与轮节点取并集：已在节点表中的地址跳过，其余补注册。 */
  for (index = 0U; index < CHASSIS_LINK_SELF_TEST_NODE_COUNT; index++)
  {
    if (MID_CANopen_GetNode(s_selfTestNodeIds[index]) != NULL)
    {
      continue;
    }

    status = MID_CANopen_AddNode(s_selfTestNodeIds[index]);
    if (status != MID_CANOPEN_OK)
    {
      return status;
    }
  }
#endif

  s_hRxSemaphore = osSemaphoreNew(APP_CHASSIS_LINK_SEMAPHORE_MAX_COUNT,
                                 APP_CHASSIS_LINK_SEMAPHORE_INIT_COUNT,
                                 &s_rxSemaphoreAttr);
  if (s_hRxSemaphore == NULL)
  {
    return MID_CANOPEN_ERROR;
  }

  s_lastValidFrameTick = HAL_GetTick();
  s_lastDispatchTick = s_lastValidFrameTick;
  s_lastDriveInitTick = s_lastValidFrameTick;
  s_isDriveReady = (APP_ChassisLink_ConfigureDrive() == MID_CANOPEN_OK) ? true : false;

  MID_UartRing_SetNotifyCallback(APP_ChassisLink_RxNotifyFromIsr);
  if (MID_UartRing_Init() != HAL_OK)
  {
    return MID_CANOPEN_ERROR;
  }

  return MID_CANOPEN_OK;
}

/**
  * @brief  串口数据到达时的中断通知，只释放信号量唤醒任务。
  * @param  None
  * @retval None
  * @warning 仅限中断上下文调用，内部不做解析、不做任何阻塞操作。
  * @note   由 MID_UartRing 的空闲中断与 DMA 满圈中断触发，真正的取数与解析在 APP_ChassisLink_Task 中完成；
  *         信号量句柄为空时静默返回；调用方中断优先级数值必须不小于
  *         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY，否则 osSemaphoreRelease 内部断言会触发。
  */
void APP_ChassisLink_RxNotifyFromIsr(void)
{
  if (s_hRxSemaphore != NULL)
  {
    (void)osSemaphoreRelease(s_hRxSemaphore);
  }
}

/**
  * @brief  底盘链路任务主体，循环完成接收与下发。
  * @param  argument  未使用。
  * @retval None
  * @note   每轮循环依次执行：等待接收信号量，带超时以免无数据时永久阻塞，
  *         超时返回同时兼作失联巡检与驱动器重试的节拍；取走环形缓冲区数据并逐字节解析成帧；
  *         调用 MID_CANopen_Process 处理心跳与 SDO 应答；按周期确保驱动器已使能；
  *         最后按失联、速度变化与刷新周期判定并下发转速。
  *         当 CHASSIS_LINK_SELF_TEST_ENABLE 置 1 时，循环末尾额外执行一次自检：先把 CHASSIS_LINK_SELF_TEST_NODE_ID_LIST
  *         中的节点逐个配置并使能，再按「转 ON_MS／停 OFF_MS」的节奏依次点动，用于确认节点地址与轮子的对应关系，
  *         该分支由宏保护，关闭时不产生任何额外代码路径。
  *         函数自带死循环，由 StartTaskHigh 直接调用，不会返回。
  */
void APP_ChassisLink_Task(void *argument)
{
  (void)argument;

  for (;;)
  {
    (void)osSemaphoreAcquire(s_hRxSemaphore, CHASSIS_LINK_POLL_TIMEOUT_MS);

    APP_ChassisLink_DrainAndParse();
    MID_CANopen_Process();
    APP_ChassisLink_EnsureDriveReady();
    APP_ChassisLink_Dispatch();

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 1U)
    APP_ChassisLink_EnsureSelfTestNodesReady();
    APP_ChassisLink_RunSelfTest();
#endif
  }
}

/**
  * @brief  把内部计数与运行状态拷出为一份统计快照。
  * @param  pStatistics  接收统计值的缓冲区指针，不可为 NULL，为空时直接返回。
  * @retval None
  * @note   接收侧的圈数、溢出次数与已读字节数实时向 MID_UartRing 查询，其余字段取自模块内静态变量，
  *         供调试器或后续上行回传使用。
  */
void APP_ChassisLink_GetStatistics(APP_ChassisLinkStatisticsTypeDef *pStatistics)
{
  if (pStatistics == NULL)
  {
    return;
  }

  pStatistics->validFrameCount = s_validFrameCount;
  pStatistics->headerMismatchCount = s_headerMismatchCount;
  pStatistics->tailMismatchCount = s_tailMismatchCount;
  pStatistics->rangeErrorCount = s_rangeErrorCount;
  pStatistics->dispatchCount = s_dispatchCount;
  pStatistics->dispatchFailureCount = s_dispatchFailureCount;
  pStatistics->lastSdoAbortCode = s_lastSdoAbortCode;
  pStatistics->commLostCount = s_commLostCount;
  pStatistics->receiveLapCount = MID_UartRing_GetLapCount();
  pStatistics->receiveOverflowCount = MID_UartRing_GetOverflowCount();
  pStatistics->receiveByteCount = MID_UartRing_GetReadBytes();
  pStatistics->targetLeftSpeed = s_targetLeftSpeed;
  pStatistics->targetRightSpeed = s_targetRightSpeed;
  pStatistics->sentLeftSpeed = s_sentLeftSpeed;
  pStatistics->sentRightSpeed = s_sentRightSpeed;
  pStatistics->isDriveReady = s_isDriveReady;
  pStatistics->isCommunicationLost = s_isCommunicationLost;
}
