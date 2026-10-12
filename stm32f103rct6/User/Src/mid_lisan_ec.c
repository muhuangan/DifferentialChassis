/**
  ******************************************************************************
  * @file    mid_lisan_ec.c
  * @brief   立三 EC 系列 CANopen 步进驱动器控制实现。
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

#include "mid_lisan_ec.h"

/**
  * @defgroup MID_LISAN_EC_Torque_Range Lisan EC Torque Range
  * @{
  */
#define MID_LISAN_EC_TORQUE_LEVEL_MIN           (-255)  /**< 力矩等级下限。 */
#define MID_LISAN_EC_TORQUE_LEVEL_MAX           (255)   /**< 力矩等级上限。 */
#define MID_LISAN_EC_MAX_TORQUE_LEVEL_MIN       0U      /**< 最大力矩等级下限。 */
#define MID_LISAN_EC_MAX_TORQUE_LEVEL_MAX       1000U   /**< 最大力矩等级上限。 */
#define MID_LISAN_EC_OFFSET_PULSE_MAX           16383U  /**< 偏移脉冲上限。 */
/**
  * @}
  */

static bool MID_LisanEc_IsModeSupported(MID_LisanEcModeTypeDef mode);

/**
  * @brief  使能驱动器。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 使能成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_EnableDrive(uint8_t nodeId)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;

  status = MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_MOTOR_FREE);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  status = MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_MOTOR_LOCK);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_ENABLE_OPERATION);
}

/**
  * @brief  写入控制字 0x6040。
  * @param  nodeId       节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                      取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  controlWord  控制字数值，写入 0x6040，取值参见 @ref MID_LISAN_EC_Control_Word。
  * @retval MID_CANOPEN_OK 写入成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_WriteControlWord(uint8_t nodeId, uint16_t controlWord)
{
  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_CONTROL_WORD, MID_CANOPEN_OD_SUB_INDEX_0, controlWord);
}

/**
  * @brief  读取控制字 0x6040。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pControlWord  指向接收 0x6040 控制字原始数值的变量的指针，取值参见 @ref MID_LISAN_EC_Control_Word，
  *                       不可为 NULL。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 指针为空。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadControlWord(uint8_t nodeId, uint16_t *pControlWord)
{
  if (pControlWord == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_ReadU16(nodeId, MID_CANOPEN_OD_CONTROL_WORD, MID_CANOPEN_OD_SUB_INDEX_0, pControlWord);
}

/**
  * @brief  电机脱机，取消锁轴状态。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMotorFree(uint8_t nodeId)
{
  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_MOTOR_FREE);
}

/**
  * @brief  电机锁机，锁定在当前位置。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMotorLock(uint8_t nodeId)
{
  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_MOTOR_LOCK);
}

/**
  * @brief  启动快速停机。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_QuickStop(uint8_t nodeId)
{
  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_QUICK_STOP);
}

/**
  * @brief  减速停止。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_Halt(uint8_t nodeId)
{
  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_HALT);
}

/**
  * @brief  复位可复位故障，清除控制字的 bit4 至 bit6。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 复位指令已发出。
  * @retval 其他          SDO 接口错误码。
  * @note   控制字 bit7 保持为 1 时其他控制指令无效，因此复位后立即清零 bit7。
  */
MID_CANopenStatusTypeDef MID_LisanEc_FaultReset(uint8_t nodeId)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;

  status = MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_FAULT_RESET);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_MOTOR_LOCK);
}

/**
  * @brief  设置工作模式 0x6060。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  mode    工作模式，取 MID_LisanEcModeTypeDef 枚举值，取值参见 @ref MID_LISAN_EC_Operation_Mode。
  * @retval MID_CANOPEN_OK            设置成功。
  * @retval MID_CANOPEN_INVALID_PARAM 模式非法。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetOperationMode(uint8_t nodeId, MID_LisanEcModeTypeDef mode)
{
  if (MID_LisanEc_IsModeSupported(mode) == false)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_WriteI8(nodeId, MID_CANOPEN_OD_MODE_OF_OPERATION, MID_CANOPEN_OD_SUB_INDEX_0, (int8_t)mode);
}

/**
  * @brief  读取当前工作模式 0x6061。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pMode   指向接收 0x6061 当前工作模式的变量的指针，取值参见 @ref MID_LISAN_EC_Operation_Mode，不可为 NULL。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 指针为空。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadOperationMode(uint8_t nodeId, MID_LisanEcModeTypeDef *pMode)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;
  int8_t mode = 0;

  if (pMode == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  status = MID_CANopen_Sdo_ReadI8(nodeId, MID_CANOPEN_OD_MODE_OF_OPERATION_DISPLAY, MID_CANOPEN_OD_SUB_INDEX_0,
                                  &mode);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  *pMode = (MID_LisanEcModeTypeDef)mode;

  return MID_CANOPEN_OK;
}

/**
  * @brief  设置电机细分 0x2001。
  * @param  nodeId         节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                        取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pulsePerRound  电机运行一圈所需脉冲数，写入 0x2001，由细分倍数与编码器线数共同决定。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMicroStep(uint8_t nodeId, uint32_t pulsePerRound)
{
  return MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_MICRO_STEP, MID_CANOPEN_OD_SUB_INDEX_0, pulsePerRound);
}

/**
  * @brief  设置驱动器峰值电流 0x2000。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  currentMa  峰值电流，单位毫安，写入 0x2000，不得超过电机额定电流。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetPeakCurrent(uint8_t nodeId, uint16_t currentMa)
{
  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_PEAK_CURRENT, MID_CANOPEN_OD_SUB_INDEX_0, currentMa);
}

/**
  * @brief  设置平滑时间 0x2002。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  smoothTimeMs  平滑时间，单位毫秒，写入 0x2002，驱动器接受的范围为 1 至 1500。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetSmoothTime(uint8_t nodeId, uint16_t smoothTimeMs)
{
  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_SMOOTH_TIME, MID_CANOPEN_OD_SUB_INDEX_0, smoothTimeMs);
}

/**
  * @brief  设置最小电流参数 0x2003。
  * @param  nodeId          节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                         取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  minimumCurrent  最小电流参数原始值，写入 0x2003；开环机型低 8 位为半流状态电流百分比，
  *                         闭环机型高低字节含义不同。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他            SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMinimumCurrent(uint8_t nodeId, uint16_t minimumCurrent)
{
  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_MINIMUM_CURRENT, MID_CANOPEN_OD_SUB_INDEX_0,
                                  minimumCurrent);
}

/**
  * @brief  设置电机运行方向 0x2051。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  direction  电机运行方向，取 MID_LISAN_EC_DIRECTION_xxx，取值参见 @ref MID_LISAN_EC_Direction。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMotorDirection(uint8_t nodeId, uint16_t direction)
{
  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_MOTOR_DIRECTION, MID_CANOPEN_OD_SUB_INDEX_0, direction);
}

/**
  * @brief  设置梯形加速度 0x6083。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  acceleration  梯形加速度，单位脉冲每平方秒，写入 0x6083。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetProfileAcceleration(uint8_t nodeId, uint32_t acceleration)
{
  return MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_PROFILE_ACCELERATION, MID_CANOPEN_OD_SUB_INDEX_0,
                                  acceleration);
}

/**
  * @brief  设置梯形减速度 0x6084。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  deceleration  梯形减速度，单位脉冲每平方秒，写入 0x6084。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetProfileDeceleration(uint8_t nodeId, uint32_t deceleration)
{
  return MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_PROFILE_DECELERATION, MID_CANOPEN_OD_SUB_INDEX_0,
                                  deceleration);
}

/**
  * @brief  设置位置模式梯形速度 0x6081。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  velocity  位置模式下允许的最大速度，单位脉冲每秒，写入 0x6081，只有大小没有方向。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetProfileVelocity(uint8_t nodeId, uint32_t velocity)
{
  return MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_PROFILE_VELOCITY, MID_CANOPEN_OD_SUB_INDEX_0, velocity);
}

/**
  * @brief  设置目标位置 0x607A。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  position  目标位置，单位脉冲，写入 0x607A。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetTargetPosition(uint8_t nodeId, int32_t position)
{
  return MID_CANopen_Sdo_WriteI32(nodeId, MID_CANOPEN_OD_TARGET_POSITION, MID_CANOPEN_OD_SUB_INDEX_0, position);
}

/**
  * @brief  设置目标速度 0x60FF。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  velocity  目标速度，单位脉冲每秒，写入 0x60FF，符号表示方向。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetTargetVelocity(uint8_t nodeId, int32_t velocity)
{
  return MID_CANopen_Sdo_WriteI32(nodeId, MID_CANOPEN_OD_TARGET_VELOCITY, MID_CANOPEN_OD_SUB_INDEX_0, velocity);
}

/**
  * @brief  设置目标力矩 0x6071。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  torque  力矩等级，写入 0x6071，数值表示大小，符号表示方向，取值参见 @ref MID_LISAN_EC_Torque_Range。
  * @retval MID_CANOPEN_OK            设置成功。
  * @retval MID_CANOPEN_INVALID_PARAM 力矩等级超出范围。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetTargetTorque(uint8_t nodeId, int16_t torque)
{
  if ((torque < MID_LISAN_EC_TORQUE_LEVEL_MIN) || (torque > MID_LISAN_EC_TORQUE_LEVEL_MAX))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_WriteI16(nodeId, MID_CANOPEN_OD_TARGET_TORQUE, MID_CANOPEN_OD_SUB_INDEX_0, torque);
}

/**
  * @brief  设置最大力矩 0x6072。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  maxTorque  最大力矩等级，写入 0x6072，数值越大可达到的最大电流越大，
  *                    取值参见 @ref MID_LISAN_EC_Torque_Range。
  * @retval MID_CANOPEN_OK            设置成功。
  * @retval MID_CANOPEN_INVALID_PARAM 力矩等级超出范围。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMaxTorque(uint8_t nodeId, uint16_t maxTorque)
{
  if ((maxTorque < MID_LISAN_EC_MAX_TORQUE_LEVEL_MIN) || (maxTorque > MID_LISAN_EC_MAX_TORQUE_LEVEL_MAX))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_MAX_TORQUE, MID_CANOPEN_OD_SUB_INDEX_0, maxTorque);
}

/**
  * @brief  配置力矩模式参数 0x2158。
  * @param  nodeId       节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                      取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  torqueMode   力矩工作方式，取 MID_LisanEcTorqueModeTypeDef 枚举值，写入 0x2158 子索引 1，
  *                      取值参见 @ref MID_LISAN_EC_Torque_Mode。
  * @param  offsetPulse  偏移脉冲，写入 0x2158 子索引 2，碰撞回原点与抓取物体时有效，
  *                      取值参见 @ref MID_LISAN_EC_Torque_Range。
  * @param  maxSpeed     力矩模式速度限制，单位转每分钟，写入 0x2158 子索引 3。
  * @retval MID_CANOPEN_OK            设置成功。
  * @retval MID_CANOPEN_INVALID_PARAM 偏移脉冲超出范围。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ConfigureTorqueMode(uint8_t nodeId, MID_LisanEcTorqueModeTypeDef torqueMode,
                                                         uint16_t offsetPulse, uint16_t maxSpeed)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;

  if (offsetPulse > MID_LISAN_EC_OFFSET_PULSE_MAX)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  status = MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_TORQUE_PARAMETER, MID_CANOPEN_OD_TORQUE_SUB_MODE,
                                    (uint16_t)torqueMode);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  status = MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_TORQUE_PARAMETER,
                                    MID_CANOPEN_OD_TORQUE_SUB_OFFSET_PULSE, offsetPulse);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_TORQUE_PARAMETER, MID_CANOPEN_OD_TORQUE_SUB_MAX_SPEED,
                                  maxSpeed);
}

/**
  * @brief  配置回原点参数。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  method        回原点方式，写入 0x6098，取值参见 @ref MID_LISAN_EC_Homing_Method。
  * @param  fastSpeed     快速回原点速度，单位脉冲每秒，写入 0x6099 子索引 1，最高位表示方向。
  * @param  slowSpeed     慢速回原点速度，单位脉冲每秒，写入 0x6099 子索引 2，最高位表示方向。
  * @param  acceleration  回原点加速度，单位脉冲每平方秒，写入 0x609A。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ConfigureHoming(uint8_t nodeId, uint8_t method, uint32_t fastSpeed,
                                                     uint32_t slowSpeed, uint32_t acceleration)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;

  status = MID_CANopen_Sdo_WriteU8(nodeId, MID_CANOPEN_OD_HOMING_METHOD, MID_CANOPEN_OD_SUB_INDEX_0, method);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  status = MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_HOMING_SPEEDS, MID_CANOPEN_OD_HOMING_SPEED_SUB_FAST,
                                    fastSpeed);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  status = MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_HOMING_SPEEDS, MID_CANOPEN_OD_HOMING_SPEED_SUB_SLOW,
                                    slowSpeed);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  return MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_HOMING_ACCELERATION, MID_CANOPEN_OD_SUB_INDEX_0,
                                  acceleration);
}

/**
  * @brief  以绝对位置方式启动轮廓位置模式运动。
  * @param  nodeId       节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                      取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  position     目标位置，单位脉冲，写入 0x607A。
  * @param  isImmediate  true 表示中断当前指令立即执行，false 表示等待当前指令执行完成后再生效。
  * @retval MID_CANOPEN_OK 指令已下发。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_MoveAbsolute(uint8_t nodeId, int32_t position, bool isImmediate)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;
  uint16_t controlWord = MID_LISAN_EC_CONTROL_WORD_ENABLE_OPERATION;

  status = MID_LisanEc_SetTargetPosition(nodeId, position);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  status = MID_LisanEc_WriteControlWord(nodeId, controlWord);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  controlWord |= MID_LISAN_EC_CONTROL_WORD_NEW_SETPOINT;
  if (isImmediate == true)
  {
    controlWord |= MID_LISAN_EC_CONTROL_WORD_SETPOINT_NOW;
  }

  return MID_LisanEc_WriteControlWord(nodeId, controlWord);
}

/**
  * @brief  以相对位置方式启动轮廓位置模式运动。
  * @param  nodeId         节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                        取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  deltaPosition  相对位移，单位脉冲，写入 0x607A，符号表示方向。
  * @param  isImmediate    true 表示中断当前指令立即执行，false 表示等待当前指令执行完成后再生效。
  * @retval MID_CANOPEN_OK 指令已下发。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_MoveRelative(uint8_t nodeId, int32_t deltaPosition, bool isImmediate)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;
  uint16_t controlWord = MID_LISAN_EC_CONTROL_WORD_ENABLE_OPERATION |
                         MID_LISAN_EC_CONTROL_WORD_RELATIVE_POSITION;

  status = MID_LisanEc_SetTargetPosition(nodeId, deltaPosition);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  status = MID_LisanEc_WriteControlWord(nodeId, controlWord);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  controlWord |= MID_LISAN_EC_CONTROL_WORD_NEW_SETPOINT;
  if (isImmediate == true)
  {
    controlWord |= MID_LISAN_EC_CONTROL_WORD_SETPOINT_NOW;
  }

  return MID_LisanEc_WriteControlWord(nodeId, controlWord);
}

/**
  * @brief  清除新的位置指令。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ClearSetpoint(uint8_t nodeId)
{
  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_ENABLE_OPERATION);
}

/**
  * @brief  启动回原点运动。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 指令已下发。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_StartHoming(uint8_t nodeId)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;

  status = MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_MOTOR_LOCK);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  status = MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_ENABLE_OPERATION);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_ENABLE_OPERATION |
                                                MID_LISAN_EC_CONTROL_WORD_NEW_SETPOINT);
}

/**
  * @brief  读取并解析状态字 0x6041。
  * @param  nodeId   节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                  取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pStatus  指向接收 0x6041 状态字解析结果的指针，取值参见 @ref MID_LISAN_EC_Status_Word，不可为 NULL。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 指针为空。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadStatusWord(uint8_t nodeId, MID_LisanEcStatusWordTypeDef *pStatus)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;
  uint16_t statusWord = 0U;

  if (pStatus == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  status = MID_CANopen_Sdo_ReadU16(nodeId, MID_CANOPEN_OD_STATUS_WORD, MID_CANOPEN_OD_SUB_INDEX_0, &statusWord);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  MID_LisanEc_ParseStatusWord(statusWord, pStatus);

  return MID_CANOPEN_OK;
}

/**
  * @brief  读取实际位置 0x6064。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pPosition  指向接收 0x6064 实际位置的变量的指针，单位脉冲，不可为 NULL。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 指针为空。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadPosition(uint8_t nodeId, int32_t *pPosition)
{
  if (pPosition == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_ReadI32(nodeId, MID_CANOPEN_OD_POSITION_ACTUAL_VALUE, MID_CANOPEN_OD_SUB_INDEX_0,
                                 pPosition);
}

/**
  * @brief  读取实际速度 0x606C。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pVelocity  指向接收 0x606C 实际速度的变量的指针，单位脉冲每秒，不可为 NULL。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 指针为空。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadVelocity(uint8_t nodeId, int32_t *pVelocity)
{
  if (pVelocity == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_ReadI32(nodeId, MID_CANOPEN_OD_VELOCITY_ACTUAL_VALUE, MID_CANOPEN_OD_SUB_INDEX_0,
                                 pVelocity);
}

/**
  * @brief  读取实时电流 0x6078。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pCurrent  指向接收 0x6078 实时电流的变量的指针，单位毫安，不可为 NULL。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 指针为空。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadCurrent(uint8_t nodeId, int16_t *pCurrent)
{
  if (pCurrent == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_ReadI16(nodeId, MID_CANOPEN_OD_CURRENT_ACTUAL_VALUE, MID_CANOPEN_OD_SUB_INDEX_0, pCurrent);
}

/**
  * @brief  读取驱动器错误代码 0x603F。
  * @param  nodeId      节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                     取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pErrorCode  指向接收 0x603F 错误代码的变量的指针，取值参见 @ref MID_LISAN_EC_Error_Code，不可为 NULL。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 指针为空。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadErrorCode(uint8_t nodeId, uint16_t *pErrorCode)
{
  if (pErrorCode == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_ReadU16(nodeId, MID_CANOPEN_OD_ERROR_CODE, MID_CANOPEN_OD_SUB_INDEX_0, pErrorCode);
}

/**
  * @brief  读取输入口与电机状态 0x60FD。
  * @param  nodeId   节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                  取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pInputs  指向接收 0x60FD 输入口与电机状态位图的变量的指针，bit0 至 bit7 对应 IN1 至 IN8，不可为 NULL。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 指针为空。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadDigitalInputs(uint8_t nodeId, uint32_t *pInputs)
{
  if (pInputs == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_ReadU32(nodeId, MID_CANOPEN_OD_DIGITAL_INPUTS, MID_CANOPEN_OD_SUB_INDEX_0, pInputs);
}

/**
  * @brief  设置从站地址 0x2150 与地址来源 0x2151。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  newAddress    新的从站地址，写入 0x2150，
  *                       合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  isFromObject  true 表示地址来源为对象字典 0x2150，false 表示拨码开关与 0x2150 之和，
  *                       取值参见 @ref MID_LISAN_EC_Address_Source。
  * @retval MID_CANOPEN_OK            设置成功。
  * @retval MID_CANOPEN_INVALID_PARAM 地址非法。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetNodeAddress(uint8_t nodeId, uint16_t newAddress, bool isFromObject)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;

  if ((newAddress < MID_CANOPEN_NODE_ID_MIN) || (newAddress > MID_CANOPEN_NODE_ID_MAX))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  status = MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_SLAVE_ADDRESS, MID_CANOPEN_OD_SUB_INDEX_0, newAddress);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_ADDRESS_SOURCE, MID_CANOPEN_OD_SUB_INDEX_0,
                                  (isFromObject == true) ? MID_LISAN_EC_ADDRESS_SOURCE_OBJECT_ONLY
                                                         : MID_LISAN_EC_ADDRESS_SOURCE_DIAL_PLUS_OBJECT);
}

/**
  * @brief  保存全部参数 0x1010 子索引 1。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 保存指令已发出。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_StoreParameters(uint8_t nodeId)
{
  return MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_STORE_PARAMETERS, MID_CANOPEN_OD_STORE_SUB_ALL,
                                  MID_CANOPEN_OD_STORE_SIGNATURE);
}

/**
  * @brief  恢复全部出厂参数 0x1011 子索引 1。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 恢复指令已发出。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_RestoreDefault(uint8_t nodeId)
{
  return MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_RESTORE_DEFAULT, MID_CANOPEN_OD_RESTORE_SUB_ALL,
                                  MID_CANOPEN_OD_RESTORE_SIGNATURE);
}

/**
  * @brief  解析状态字得到 CiA402 驱动状态。
  * @param  statusWord  状态字 0x6041 原始数值，取值参见 @ref MID_LISAN_EC_Status_Word。
  * @retval 驱动状态。
  */
MID_LisanEcStateTypeDef MID_LisanEc_ParseState(uint16_t statusWord)
{
  uint16_t lowNibble = (uint16_t)(statusWord & MID_LISAN_EC_STATE_MASK_LOW);
  uint16_t full = (uint16_t)(statusWord & MID_LISAN_EC_STATE_MASK_FULL);

  if (full == MID_LISAN_EC_STATE_CODE_OPERATION_ENABLED)
  {
    return MID_LISAN_EC_STATE_OPERATION_ENABLED;
  }

  if (full == MID_LISAN_EC_STATE_CODE_QUICK_STOP_ACTIVE)
  {
    return MID_LISAN_EC_STATE_QUICK_STOP_ACTIVE;
  }

  if (full == MID_LISAN_EC_STATE_CODE_SWITCHED_ON)
  {
    return MID_LISAN_EC_STATE_SWITCHED_ON;
  }

  if (full == MID_LISAN_EC_STATE_CODE_READY_TO_SWITCH_ON)
  {
    return MID_LISAN_EC_STATE_READY_TO_SWITCH_ON;
  }

  if (lowNibble == MID_LISAN_EC_STATE_CODE_FAULT_REACTION_ACTIVE)
  {
    return MID_LISAN_EC_STATE_FAULT_REACTION_ACTIVE;
  }

  if (lowNibble == MID_LISAN_EC_STATE_CODE_FAULT)
  {
    return MID_LISAN_EC_STATE_FAULT;
  }

  if (lowNibble == MID_LISAN_EC_STATE_CODE_SWITCH_ON_DISABLED)
  {
    return MID_LISAN_EC_STATE_SWITCH_ON_DISABLED;
  }

  if (lowNibble == MID_LISAN_EC_STATE_CODE_NOT_READY_TO_SWITCH_ON)
  {
    return MID_LISAN_EC_STATE_NOT_READY_TO_SWITCH_ON;
  }

  return MID_LISAN_EC_STATE_UNKNOWN;
}

/**
  * @brief  解析状态字到结构体。
  * @param  statusWord  状态字 0x6041 原始数值，取值参见 @ref MID_LISAN_EC_Status_Word。
  * @param  pStatus     指向接收解析结果的指针，成员与 0x6041 各位一一对应，不可为 NULL。
  * @retval None
  */
void MID_LisanEc_ParseStatusWord(uint16_t statusWord, MID_LisanEcStatusWordTypeDef *pStatus)
{
  if (pStatus == NULL)
  {
    return;
  }

  pStatus->raw = statusWord;
  pStatus->isFaultFree = ((statusWord & MID_LISAN_EC_STATUS_WORD_FAULT_FREE) != 0U) ? true : false;
  pStatus->isReadyToSwitchOn = ((statusWord & MID_LISAN_EC_STATUS_WORD_READY_TO_SWITCH_ON) != 0U) ? true : false;
  pStatus->isOperationEnabled = ((statusWord & MID_LISAN_EC_STATUS_WORD_OPERATION_ENABLED) != 0U) ? true : false;
  pStatus->isFault = ((statusWord & MID_LISAN_EC_STATUS_WORD_FAULT) != 0U) ? true : false;
  pStatus->isVoltageEnabled = ((statusWord & MID_LISAN_EC_STATUS_WORD_VOLTAGE_ENABLED) != 0U) ? true : false;
  pStatus->isQuickStopActive = ((statusWord & MID_LISAN_EC_STATUS_WORD_QUICK_STOP) == 0U) ? true : false;
  pStatus->isSwitchOnDisabled = ((statusWord & MID_LISAN_EC_STATUS_WORD_SWITCH_ON_DISABLED) != 0U) ? true : false;
  pStatus->isWarning = ((statusWord & MID_LISAN_EC_STATUS_WORD_WARNING) != 0U) ? true : false;
  pStatus->isRemote = ((statusWord & MID_LISAN_EC_STATUS_WORD_REMOTE) != 0U) ? true : false;
  pStatus->isTargetReached = ((statusWord & MID_LISAN_EC_STATUS_WORD_TARGET_REACHED) != 0U) ? true : false;
  pStatus->isInternalLimitActive = ((statusWord & MID_LISAN_EC_STATUS_WORD_INTERNAL_LIMIT) != 0U) ? true : false;
  pStatus->isHomingAttained = ((statusWord & MID_LISAN_EC_STATUS_WORD_HOMING_ATTAINED) != 0U) ? true : false;
}

/**
  * @brief  将错误代码转换为可读文本。
  * @param  errorCode  错误代码 0x603F，取值参见 @ref MID_LISAN_EC_Error_Code。
  * @retval 错误文本，无故障返回 no error，未知代码返回 unknown。
  */
const char *MID_LisanEc_ErrorCodeToString(uint16_t errorCode)
{
  switch (errorCode)
  {
    case 0x0000U:
      return "no error";
    case MID_LISAN_EC_ERROR_CODE_OVER_CURRENT:
      return "driver over current";
    case MID_LISAN_EC_ERROR_CODE_MOTOR_OPEN:
      return "motor phase open";
    case MID_LISAN_EC_ERROR_CODE_VOLTAGE_TOO_HIGH:
      return "supply voltage too high";
    case MID_LISAN_EC_ERROR_CODE_VOLTAGE_TOO_LOW:
      return "supply voltage too low";
    case MID_LISAN_EC_ERROR_CODE_PHASE_A_DRIFT:
      return "phase A voltage drift";
    case MID_LISAN_EC_ERROR_CODE_PHASE_B_DRIFT:
      return "phase B voltage drift";
    case MID_LISAN_EC_ERROR_CODE_POSITION_ERROR:
      return "position error";
    case MID_LISAN_EC_ERROR_CODE_ENCODER_ERROR:
      return "encoder error";
    default:
      return "unknown";
  }
}

/**
  * @brief  判断工作模式是否为驱动器支持的模式。
  * @param  mode  待判定的工作模式，取 MID_LisanEcModeTypeDef 枚举值，取值参见 @ref MID_LISAN_EC_Operation_Mode。
  * @retval true   驱动器支持该模式。
  * @retval false  驱动器不支持该模式。
  */
static bool MID_LisanEc_IsModeSupported(MID_LisanEcModeTypeDef mode)
{
  if ((mode == MID_LISAN_EC_MODE_PROFILE_POSITION) || (mode == MID_LISAN_EC_MODE_PROFILE_VELOCITY) ||
      (mode == MID_LISAN_EC_MODE_PROFILE_TORQUE) || (mode == MID_LISAN_EC_MODE_HOMING))
  {
    return true;
  }

  return false;
}
