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
#include "app_motor_startup.h"

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

#if (CHASSIS_LINK_FRAME_PARSE_ACTIVE == 1U)
static uint8_t s_parseBuffer[CHASSIS_LINK_PARSE_CHUNK_BYTES];   /**< 从环形缓冲区批量取数的中转缓冲区。 */
#endif

#if (CHASSIS_LINK_UPPER_SIM_ACTIVE == 1U)
static uint8_t  s_simFrame[MID_CHASSIS_FRAME_LENGTH_BYTES]; /**< 按协议布局拼好待注入环形缓冲区的报文。 */
static uint32_t s_simLastTick = 0U;                        /**< 最近一次注入模拟报文的时刻，单位毫秒。 */
static int32_t  s_simLeftSpeed = 0;                        /**< 左轮三角波当前转速，单位转每分。 */
static int32_t  s_simRightSpeed = 0;                       /**< 右轮三角波当前转速，单位转每分。 */
static bool     s_isSimLeftRising = false;                 /**< 左轮三角波当前处于升速段还是降速段。 */
static bool     s_isSimRightRising = false;                /**< 右轮三角波当前处于升速段还是降速段。 */
#endif

/* 左右轮节点地址组，顺序由 CHASSIS_LINK_WHEEL_NODE_ID_LIST 决定，取用下标统一用 CHASSIS_LINK_WHEEL_INDEX_* 表达。 */
static const uint8_t s_wheelNodeIds[CHASSIS_LINK_WHEEL_COUNT] = CHASSIS_LINK_WHEEL_NODE_ID_LIST;

static int32_t  s_targetLeftSpeed = 0;           /**< 上位机最新下发的左轮目标速度，单位脉冲每秒。 */
static int32_t  s_targetRightSpeed = 0;          /**< 上位机最新下发的右轮目标速度，单位脉冲每秒。 */
static int32_t  s_sentLeftSpeed = 0;             /**< 最近一次实际下发的左轮速度，单位脉冲每秒。 */
static int32_t  s_sentRightSpeed = 0;            /**< 最近一次实际下发的右轮速度，单位脉冲每秒。 */
static uint32_t s_lastDispatchTick = 0U;         /**< 最近一次下发速度的时间戳，单位毫秒。 */
static bool     s_hasValidFrame = false;         /**< 是否收到过至少一帧合法数据。 */
static bool     s_isDriveReady = false;          /**< 两轮驱动器是否已完成模式设置与使能。 */

static volatile uint32_t s_validFrameCount = 0U;         /**< 合法帧累计数。 */
static volatile uint32_t s_headerMismatchCount = 0U;     /**< 帧头失配累计数。 */
static volatile uint32_t s_tailMismatchCount = 0U;       /**< 帧尾失配累计数。 */
static volatile uint32_t s_rangeErrorCount = 0U;         /**< 范围校验失败累计数。 */
static volatile uint32_t s_dispatchCount = 0U;           /**< 单轮转速下发成功累计数。 */
static volatile uint32_t s_dispatchFailureCount = 0U;    /**< 转速下发失败累计数。 */
static volatile uint32_t s_lastSdoAbortCode = 0U;        /**< 最近一次下发失败对应节点的 SDO 中止码。 */

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
static uint32_t s_pulsesPerWheelRev[CHASSIS_LINK_WHEEL_COUNT]; /**< 左右轮各转一圈对应的脉冲数，取 2001h 每电机圈脉冲数乘减速比，用于把上位机 rpm 换算成 60FFh 的脉冲每秒。 */
#endif

#if (CHASSIS_LINK_SPIN_TEST_ACTIVE == 1)
/* 自检节点地址组，顺序由 CHASSIS_LINK_SELF_TEST_NODE_ID_LIST 决定，点动时按下标依次轮转。 */
static const uint8_t s_selfTestNodeIds[CHASSIS_LINK_SELF_TEST_NODE_COUNT] = CHASSIS_LINK_SELF_TEST_NODE_ID_LIST;



#endif

static void APP_ChassisLink_EnsureDriveReady(void);
#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
static void APP_ChassisLink_HandleByte(uint8_t byte);
static void APP_ChassisLink_OnValidFrame(const MID_ChassisPayloadTypeDef *pPayload);
#endif
#if (CHASSIS_LINK_FRAME_PARSE_ACTIVE == 1U)
static void APP_ChassisLink_DrainAndParse(void);
#endif
#if (CHASSIS_LINK_UPPER_SIM_ACTIVE == 1U)
static void APP_ChassisLink_PackSimFrame(uint8_t *pFrame, int32_t leftRpm, int32_t rightRpm);
static void APP_ChassisLink_AdvanceSimWave(void);
static void APP_ChassisLink_InjectSimFrame(void);
#endif
#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
static void APP_ChassisLink_Dispatch(void);
#endif
#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
static void APP_ChassisLink_SendSpeeds(int32_t leftSpeed, int32_t rightSpeed);
#endif
#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
static void APP_ChassisLink_SendOneWheel(uint8_t wheelIndex, int32_t speed);
#endif
#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
static void APP_ChassisLink_RecordFailure(uint8_t nodeId);
#endif
static int32_t APP_ChassisLink_ApplyInvert(int32_t speed, uint32_t invertFlag);
#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
static bool APP_ChassisLink_IsSpeedChanged(int32_t target, int32_t sent);
#endif

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 1U)
static void APP_ChassisLink_RunSelfTest(void);
#endif

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

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
/**
  * @brief  把电机每圈脉冲数换算到轮子一圈的脉冲数。
  * @param  motorPulses  驱动器 2001h 读回的电机每圈脉冲数，即细分。
  * @retval 轮子一圈对应的脉冲数。
  * @note   减速比取 CHASSIS_LINK_GEAR_NUMERATOR 与 CHASSIS_LINK_GEAR_DENOMINATOR；
  *         用 64 位中间量先乘后除并做四舍五入，避免大细分时溢出。
  */
static uint32_t APP_ChassisLink_ScaleToWheel(uint32_t motorPulses)
{
  return (uint32_t) (((uint64_t) motorPulses * CHASSIS_LINK_GEAR_NUMERATOR +
                      (CHASSIS_LINK_GEAR_DENOMINATOR / 2U)) / CHASSIS_LINK_GEAR_DENOMINATOR);
}

/**
  * @brief  启动阶段读入左右轮各转一圈对应的脉冲数。
  * @param  None
  * @retval None
  * @note   对每个轮节点读一次 2001h；读取失败或读回 0 时改用
  *         CHASSIS_LINK_PULSES_PER_MOTOR_REV_DEFAULT 乘减速比，保证换算始终有可用基准。
  *         本函数在调度器启动前调用，失败只影响换算基准，不阻塞启动流程。
  */
static void APP_ChassisLink_ReadPulsesPerRev(void)
{
  uint8_t  index;
  uint32_t motorPulses = 0U;

  for (index = 0U; index < CHASSIS_LINK_WHEEL_COUNT; index++)
  {
    if ((MID_CANopen_Sdo_ReadU32(s_wheelNodeIds[index], MID_CANOPEN_OD_MICRO_STEP,
                                 MID_CANOPEN_OD_SUB_INDEX_0, &motorPulses) == MID_CANOPEN_OK) &&
        (motorPulses != 0U))
    {
      s_pulsesPerWheelRev[index] = APP_ChassisLink_ScaleToWheel(motorPulses);
    }
    else
    {
      s_pulsesPerWheelRev[index] = APP_ChassisLink_ScaleToWheel(CHASSIS_LINK_PULSES_PER_MOTOR_REV_DEFAULT);
    }
  }
}

/**
  * @brief  把上位机下发的轮子转速换算成目标速度对象 0x60FF 的脉冲每秒。
  * @param  rpm           轮子目标转速，单位转每分，即帧内原始数值。
  * @param  pulsesPerRev  该轮一圈对应的脉冲数，由启动阶段读入。
  * @retval 目标速度，单位脉冲每秒。
  * @note   换算式为 转速 乘 每圈脉冲数 除 60，先乘后除并用 64 位中间量，结果按 int32 截断；
  *         默认 4000 细分与 10:1 减速比下 1 转每分约为 667 脉冲每秒。
  */
static int32_t APP_ChassisLink_RpmToPps(int32_t rpm, uint32_t pulsesPerRev)
{
  int64_t pps = ((int64_t) rpm * (int64_t) pulsesPerRev) / 60;

  if (pps > (int64_t) INT32_MAX)
  {
    return INT32_MAX;
  }

  if (pps < (int64_t) INT32_MIN)
  {
    return INT32_MIN;
  }

  return (int32_t) pps;
}
#endif

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
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

#endif

/**
  * @brief  记录一次转速下发失败。
  * @param  nodeId  下发失败的节点地址。
  * @retval None
  * @note   失败计数累加，并把该节点最近一次的 SDO 中止码拷入 s_lastSdoAbortCode，
  *         便于通过统计快照定位失败原因。
  */
#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
static void APP_ChassisLink_RecordFailure(uint8_t nodeId)
{
  MID_CANopenNodeTypeDef *pNode = MID_CANopen_GetNode(nodeId);

  s_dispatchFailureCount++;

  if (pNode != NULL)
  {
    s_lastSdoAbortCode = pNode->lastSdoAbortCode;
  }
}
#endif

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
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

#endif

/**
  * @brief  用一帧合法数据刷新目标速度与接收时刻。
  * @param  pPayload  已解析出的负载，不可为 NULL。
  * @retval None
  * @note   帧内两个字段是轮子目标转速，单位转每分，先按每轮脉冲数换算成 60FFh 的脉冲每秒，
  *         再按 CHASSIS_LINK_SPEED_INVERT_LEFT 与 CHASSIS_LINK_SPEED_INVERT_RIGHT 统一方向；
  *         同时累加合法帧计数并刷新最近接收时刻，供失联判定使用。
  */
#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
static void APP_ChassisLink_OnValidFrame(const MID_ChassisPayloadTypeDef *pPayload)
{
  s_validFrameCount++;
  s_targetLeftSpeed = APP_ChassisLink_ApplyInvert(
    APP_ChassisLink_RpmToPps(pPayload->leftSpeed, s_pulsesPerWheelRev[CHASSIS_LINK_WHEEL_INDEX_LEFT]),
    CHASSIS_LINK_SPEED_INVERT_LEFT);
  s_targetRightSpeed = APP_ChassisLink_ApplyInvert(
    APP_ChassisLink_RpmToPps(pPayload->rightSpeed, s_pulsesPerWheelRev[CHASSIS_LINK_WHEEL_INDEX_RIGHT]),
    CHASSIS_LINK_SPEED_INVERT_RIGHT);
  s_hasValidFrame = true;
}
#endif

/**
  * @brief  把一个字节投喂给解析器，并按解析结果刷新负载或累加统计计数。
  * @param  byte  待解析字节。
  * @retval None
  * @note   串口接收与上位机帧模拟两个来源共用本函数，保证两者对解析结果的处理完全一致，
  *         统计计数与负载刷新语义不会因为数据来源不同而出现差异。
  */
#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
static void APP_ChassisLink_HandleByte(uint8_t byte)
{
  MID_ChassisPayloadTypeDef payload;

  switch (MID_ChassisProtocol_ParseByte(byte, &payload))
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
#endif

#if (CHASSIS_LINK_FRAME_PARSE_ACTIVE == 1U)
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
      APP_ChassisLink_HandleByte(s_parseBuffer[index]);
    }

    budget = (uint16_t)(budget - count);
    available = MID_UartRing_GetAvailable();
  }
}
#endif

#if (CHASSIS_LINK_UPPER_SIM_ACTIVE == 1U)
/**
  * @brief  按定长协议把左右轮转速拼成一帧与上位机同格式的模拟报文。
  * @param  pFrame    帧缓冲首地址，长度不小于 MID_CHASSIS_FRAME_LENGTH_BYTES，不可为 NULL。
  * @param  leftRpm   左轮目标转速，单位转每分，符号表示方向。
  * @param  rightRpm  右轮目标转速，单位转每分，符号表示方向。
  * @retval None
  * @note   布局与真实上位机一致：两字节帧头、两个小端 int32 转速、两字节帧尾；
  *         偏移与帧头帧尾取值全部取自 mid_chassis_protocol.h 的协议常量，不另写数值，
  *         协议若调整，本函数自动跟随，无需同步修改。
  */
static void APP_ChassisLink_PackSimFrame(uint8_t *pFrame, int32_t leftRpm, int32_t rightRpm)
{
  uint8_t byteIndex = 0U;

  if (pFrame == NULL)
  {
    return;
  }

  pFrame[MID_CHASSIS_FRAME_OFFSET_HEADER] = MID_CHASSIS_FRAME_HEADER_0;
  pFrame[MID_CHASSIS_FRAME_OFFSET_HEADER_1] = MID_CHASSIS_FRAME_HEADER_1;

  /* 两个转速字段各占一个 int32，逐字节取小端表示，与上位机的 struct.pack('<i') 字节序一致。 */
  for (byteIndex = 0U; byteIndex < (uint8_t)sizeof(int32_t); byteIndex++)
  {
    pFrame[MID_CHASSIS_FRAME_OFFSET_LEFT_SPEED + byteIndex] =
      (uint8_t)(((uint32_t)leftRpm) >> (byteIndex * 8U));
    pFrame[MID_CHASSIS_FRAME_OFFSET_RIGHT_SPEED + byteIndex] =
      (uint8_t)(((uint32_t)rightRpm) >> (byteIndex * 8U));
  }

  pFrame[MID_CHASSIS_FRAME_OFFSET_TAIL] = MID_CHASSIS_FRAME_TAIL_0;
  pFrame[MID_CHASSIS_FRAME_OFFSET_TAIL_1] = MID_CHASSIS_FRAME_TAIL_1;
}

/**
  * @brief  把左右轮三角波各推进一个步进。
  * @param  None
  * @retval None
  * @note   每轮方向相反的轮子各自独立翻转，轮子在速度到达上下限时换向并立即反向，
  *         因此持续运行也不会停在端点上；步进量取自 CHASSIS_LINK_UPPER_SIM_RAMP_RPM，
  *         配合发送周期决定斜坡快慢。三角波只决定目标转速，不参与任何限幅判定，
  *         区间配置超出限幅时由解析器按真实链路的方式整帧拒绝。
  */
static void APP_ChassisLink_AdvanceSimWave(void)
{
  if (s_isSimLeftRising != false)
  {
    s_simLeftSpeed += (int32_t)CHASSIS_LINK_UPPER_SIM_RAMP_RPM;
    if (s_simLeftSpeed >= (int32_t)CHASSIS_LINK_UPPER_SIM_LEFT_MAX_RPM)
    {
      s_simLeftSpeed = (int32_t)CHASSIS_LINK_UPPER_SIM_LEFT_MAX_RPM;
      s_isSimLeftRising = false;
    }
  }
  else
  {
    s_simLeftSpeed -= (int32_t)CHASSIS_LINK_UPPER_SIM_RAMP_RPM;
    if (s_simLeftSpeed <= (int32_t)CHASSIS_LINK_UPPER_SIM_LEFT_MIN_RPM)
    {
      s_simLeftSpeed = (int32_t)CHASSIS_LINK_UPPER_SIM_LEFT_MIN_RPM;
      s_isSimLeftRising = true;
    }
  }

  if (s_isSimRightRising != false)
  {
    s_simRightSpeed += (int32_t)CHASSIS_LINK_UPPER_SIM_RAMP_RPM;
    if (s_simRightSpeed >= (int32_t)CHASSIS_LINK_UPPER_SIM_RIGHT_MAX_RPM)
    {
      s_simRightSpeed = (int32_t)CHASSIS_LINK_UPPER_SIM_RIGHT_MAX_RPM;
      s_isSimRightRising = false;
    }
  }
  else
  {
    s_simRightSpeed -= (int32_t)CHASSIS_LINK_UPPER_SIM_RAMP_RPM;
    if (s_simRightSpeed <= (int32_t)CHASSIS_LINK_UPPER_SIM_RIGHT_MIN_RPM)
    {
      s_simRightSpeed = (int32_t)CHASSIS_LINK_UPPER_SIM_RIGHT_MIN_RPM;
      s_isSimRightRising = true;
    }
  }
}

/**
  * @brief  按固定周期推进三角波并把合成报文注入环形缓冲区。
  * @param  None
  * @retval None
  * @note   本函数只负责产生字节，写入位置与真实串口完全相同的环形缓冲区；
  *         随后仍由 APP_ChassisLink_DrainAndParse 取数并解析，不直接调用解析器，
  *         因此取数、解析、限幅、统计这条路径的覆盖范围与接真上位机一致。
  *         注入时触发一次数据到达通知，等价于真实串口的一次空闲中断。
  */
static void APP_ChassisLink_InjectSimFrame(void)
{
  uint32_t currentTick = HAL_GetTick();

  if ((uint32_t)(currentTick - s_simLastTick) < CHASSIS_LINK_UPPER_SIM_PERIOD_MS)
  {
    return;
  }

  s_simLastTick = currentTick;

  APP_ChassisLink_AdvanceSimWave();
  APP_ChassisLink_PackSimFrame(s_simFrame, s_simLeftSpeed, s_simRightSpeed);

  (void)MID_UartRing_Inject(s_simFrame, MID_CHASSIS_FRAME_LENGTH_BYTES);
}
#endif

/**
  * @brief  汇总独立节点状态机的实际启动验证结果。
  * @note   成功节点保持使能，失败节点独立重试，不重复清零已运行轮子的速度。
  */
static void APP_ChassisLink_EnsureDriveReady(void)
{
  uint8_t index = 0U;
  s_isDriveReady = true;
  for (index = 0U; index < CHASSIS_LINK_WHEEL_COUNT; index++)
  {
    if (!APP_MotorStartup_IsReady(s_wheelNodeIds[index]))
    {
      s_isDriveReady = false;
    }
  }
}

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 1U)
/**
  * @brief  根据配置推进位置点动、单轮速度点动或双轮一圈测试。
  * @note   双轮一圈模式等待两轮就绪并按位置反馈分别停车；逐轮模式让单个节点持续正转，转满配置时长后轮换。
  *         自检期间不执行上位机速度下发，避免两个流程覆盖目标速度。
  */
static void APP_ChassisLink_RunSelfTest(void)
{
#if CHASSIS_LINK_POSITION_TEST_ACTIVE
  APP_MotorStartup_PositionTestProcess();
#else
#if CHASSIS_LINK_SYNC_ROUND_ACTIVE
  APP_MotorStartup_SyncRoundProcess(s_selfTestNodeIds, CHASSIS_LINK_SELF_TEST_NODE_COUNT);
#else
  APP_MotorStartup_VelocityTestProcess(s_selfTestNodeIds, CHASSIS_LINK_SELF_TEST_NODE_COUNT);
#endif
#endif
}
#endif

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
/**
  * @brief  按当前目标速度决定是否下发转速。
  * @param  None
  * @retval None
  * @note   触发条件有两项：目标速度相对已下发速度超出死区；距上次下发达到 CHASSIS_LINK_REFRESH_PERIOD_MS。
  *         没有收到过合法帧时把两轮目标强制归零，同样按刷新周期持续下发零速，
  *         保证上位机不发数据时两轮立刻停住，不会停留在上一次的非零目标上；
  *         驱动器未就绪时无法安全写入速度对象，直接返回。
  */
static void APP_ChassisLink_Dispatch(void)
{
  uint32_t currentTick = HAL_GetTick();
  bool needDispatch = false;

  if (s_isDriveReady == false)
  {
    return;
  }

  if (s_hasValidFrame == false)
  {
    s_targetLeftSpeed = 0;
    s_targetRightSpeed = 0;
  }

  if ((APP_ChassisLink_IsSpeedChanged(s_targetLeftSpeed, s_sentLeftSpeed) != false) ||
      (APP_ChassisLink_IsSpeedChanged(s_targetRightSpeed, s_sentRightSpeed) != false))
  {
    needDispatch = true;
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

#endif

/**
  * @brief  初始化底盘链路，须在调度器启动前调用。
  * @param  None
  * @retval MID_CANOPEN_OK            初始化成功。
  * @retval MID_CANOPEN_INVALID_PARAM CAN 句柄非法。
  * @retval MID_CANOPEN_ERROR         协议栈、节点注册、信号量或串口接收启动失败。
  * @note   执行顺序：初始化 CANopen 协议栈；注册全部轮节点，并在自检开关打开时补注册自检节点；
  *         读入左右轮各转一圈的脉冲数作为 rpm 换算基准，读不到 2001h 时回落到配置默认细分；
  *         创建绑定静态控制块的二值信号量；
  *         注册独立启动诊断上下文，配置与使能由任务周期执行并验证；
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

#if (CHASSIS_LINK_SPIN_TEST_ACTIVE == 1)
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

  for (index = 0U; index < CHASSIS_LINK_WHEEL_COUNT; index++)
  {
    status = APP_MotorStartup_AddNode(s_wheelNodeIds[index]);
    if (status != MID_CANOPEN_OK)
    {
      return status;
    }
  }
#if (CHASSIS_LINK_SPIN_TEST_ACTIVE == 1)
  for (index = 0U; index < CHASSIS_LINK_SELF_TEST_NODE_COUNT; index++)
  {
    status = APP_MotorStartup_AddNode(s_selfTestNodeIds[index]);
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

  s_lastDispatchTick = HAL_GetTick();
  s_isDriveReady = false;

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 0U)
  /* 换算基准先落地：读不到 2001h 时内部回落到默认细分，之后每帧都按它把 rpm 换成脉冲每秒。 */
  APP_ChassisLink_ReadPulsesPerRev();
#endif

#if (CHASSIS_LINK_UPPER_SIM_ACTIVE == 1U)
  /* 三角波从下限起步并先升速，首帧即可观察到加速过程。
     注入模式替代 UART4 的 DMA 写入，环形缓冲区本身与真实接收共用同一个。 */
  s_simLeftSpeed = (int32_t)CHASSIS_LINK_UPPER_SIM_LEFT_MIN_RPM;
  s_simRightSpeed = (int32_t)CHASSIS_LINK_UPPER_SIM_RIGHT_MIN_RPM;
  s_isSimLeftRising = true;
  s_isSimRightRising = true;
  s_simLastTick = HAL_GetTick();
  APP_ChassisLink_PackSimFrame(s_simFrame, s_simLeftSpeed, s_simRightSpeed);
#endif

  MID_UartRing_SetNotifyCallback(APP_ChassisLink_RxNotifyFromIsr);
#if (CHASSIS_LINK_UPPER_SIM_ACTIVE == 1U)
  if (MID_UartRing_StartInject() != HAL_OK)
  {
    return MID_CANOPEN_ERROR;
  }
#else
  if (MID_UartRing_Init() != HAL_OK)
  {
    return MID_CANOPEN_ERROR;
  }
#endif

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
  * @note   每轮等待接收信号量、取数并解析、更新 CAN 状态并推进一次启动诊断阶段。
  *         自检模式只执行带停止验证的点动；正常模式在两轮启动验证完成后下发速度。
  *         开启上位机帧模拟时，字节来源由 UART4 的 DMA 换成任务内合成的三角波报文，
  *         但取数仍走同一个环形缓冲区、解析仍走同一个解析器，后续流程完全一致。
  *         函数由 StartTaskHigh 调用，启动历史保存在 g_motorStartupDebug。
  */
void APP_ChassisLink_Task(void *argument)
{
  (void)argument;

  for (;;)
  {
    (void)osSemaphoreAcquire(s_hRxSemaphore, CHASSIS_LINK_POLL_TIMEOUT_MS);

#if (CHASSIS_LINK_FRAME_PARSE_ACTIVE == 1U)
#if (CHASSIS_LINK_UPPER_SIM_ACTIVE == 1U)
    APP_ChassisLink_InjectSimFrame();
#endif
    APP_ChassisLink_DrainAndParse();
#endif
    MID_CANopen_Process();
    APP_MotorStartup_Process();
    APP_ChassisLink_EnsureDriveReady();

#if (CHASSIS_LINK_SELF_TEST_ENABLE == 1U)
    APP_ChassisLink_RunSelfTest();
#else
    APP_ChassisLink_Dispatch();
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
  pStatistics->receiveLapCount = MID_UartRing_GetLapCount();
  pStatistics->receiveOverflowCount = MID_UartRing_GetOverflowCount();
  pStatistics->receiveByteCount = MID_UartRing_GetReadBytes();
  pStatistics->targetLeftSpeed = s_targetLeftSpeed;
  pStatistics->targetRightSpeed = s_targetRightSpeed;
  pStatistics->sentLeftSpeed = s_sentLeftSpeed;
  pStatistics->sentRightSpeed = s_sentRightSpeed;
  pStatistics->isDriveReady = s_isDriveReady;
  pStatistics->hasValidFrame = s_hasValidFrame;
}
