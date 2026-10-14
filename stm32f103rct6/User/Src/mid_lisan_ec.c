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

/* 状态字解析只在本文件内使用，前置声明供 ReadStatusWord 调用。 */
static void MID_LisanEc_ParseStatusWord(uint16_t statusWord, MID_LisanEcStatusWordTypeDef *pStatus);

#if (MID_LISAN_EC_ENABLE_STATE_CMDS == 1)
/**
  * @brief  使能驱动器。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 使能成功。
  * @retval 其他          SDO 接口错误码。
  * @note   按 CiA402 上电顺序依次写入控制字：先 0x06 取消锁轴，再 0x07 锁机，最后 0x0F 通电运行；
  *         任一步失败立即返回，不再继续。
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
#endif


/**
  * @brief  写入控制字 0x6040。
  * @param  nodeId       节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                      取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  controlWord  控制字数值，写入 0x6040，取值参见 @ref MID_LISAN_EC_Control_Word。
  * @retval MID_CANOPEN_OK 写入成功。
  * @retval 其他          SDO 接口错误码。
  * @note   直接把控制字写进对象 0x6040 子索引 0，不做数值校验。
  */
MID_CANopenStatusTypeDef MID_LisanEc_WriteControlWord(uint8_t nodeId, uint16_t controlWord)
{
  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_CONTROL_WORD, MID_CANOPEN_OD_SUB_INDEX_0, controlWord);
}

#if (MID_LISAN_EC_ENABLE_STATE_CMDS == 1)
/**
  * @brief  读取控制字 0x6040。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pControlWord  指向接收 0x6040 控制字原始数值的变量的指针，取值参见 @ref MID_LISAN_EC_Control_Word，
  *                       不可为 NULL。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 指针为空。
  * @retval 其他                      SDO 接口错误码。
  * @note   指针为空判参数非法，否则读取对象 0x6040 子索引 0 的原始数值。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadControlWord(uint8_t nodeId, uint16_t *pControlWord)
{
  if (pControlWord == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_ReadU16(nodeId, MID_CANOPEN_OD_CONTROL_WORD, MID_CANOPEN_OD_SUB_INDEX_0, pControlWord);
}
#endif


#if (MID_LISAN_EC_ENABLE_STATE_CMDS == 1)
/**
  * @brief  电机脱机，取消锁轴状态。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  * @note   写入控制字 0x0006，bit1 置位，取消锁轴使电机自由。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMotorFree(uint8_t nodeId)
{
  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_MOTOR_FREE);
}
#endif


#if (MID_LISAN_EC_ENABLE_STATE_CMDS == 1)
/**
  * @brief  电机锁机，锁定在当前位置。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  * @note   写入控制字 0x0007，bit0 至 bit2 置位，把电机锁定在当前位置。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMotorLock(uint8_t nodeId)
{
  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_MOTOR_LOCK);
}
#endif


#if (MID_LISAN_EC_ENABLE_STATE_CMDS == 1)
/**
  * @brief  启动快速停机。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  * @note   写入控制字 0x0002，满足手册 4.1.2（p26）对快速停机给出的（**** **** 0*** *01*）条件：
  *         bit1 为 1、bit2 与 bit7 为 0；该值即手册 4.1.1（p25）状态切换 11「伺服运行→快速停机」。
  *         手册 4.2.2（p37）汇总表把快速停止写作 00Bh，与 4.1.1 及 PP/PV/HM 全部示例报文不一致，
  *         且 00Bh 不属于 4.1.1 列出的任何一条状态切换命令，故按 4.1.1 取 0x0002。
  */
MID_CANopenStatusTypeDef MID_LisanEc_QuickStop(uint8_t nodeId)
{
  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_QUICK_STOP);
}
#endif


#if (MID_LISAN_EC_ENABLE_STATE_CMDS == 1)
/**
  * @brief  减速停止。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  * @note   写入控制字 0x010F，由运行命令 0x0F（bit0~bit3 构成「接通主电允许运行」）叠加 bit8 组成，
  *         对应手册 4.2.2（p37）与 4.2.4（p50）汇总表的 010Fh。
  *         手册 PP/PV/HM 示例报文（p34、p35、p48）写成 0x0100，但其 bit0~bit3 为 0000，
  *         按手册 4.1.2（p26）「bit0~bit3 必须共同构成一条命令」会被解释为脱机命令而非减速停止，
  *         故不采用；0x010F 可保证整个减速过程驱动器仍处于伺服运行状态。
  *         停止后如需恢复锁轴，写回 0x000F 清除 bit8 即可（手册 4.1.2 bit8 说明，p26），
  *         电机运行状态不会自动恢复，须重新走一遍状态机。
  */
MID_CANopenStatusTypeDef MID_LisanEc_Halt(uint8_t nodeId)
{
  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_HALT);
}
#endif


#if (MID_LISAN_EC_ENABLE_STATE_CMDS == 1)
/**
  * @brief  复位可复位故障，并把控制字 bit7 清零。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 复位指令已发出。
  * @retval 其他          SDO 接口错误码。
  * @note   先写 0x0080，由 bit7 上升沿触发故障复位（手册 4.1.1 状态切换 15，p25）；
  *         手册 4.1.2（p26）说明 bit7 保持为 1 时其他控制指令均无效，因此复位后必须立即把 bit7 清零。
  *         复位完成后驱动器停在「伺服无故障」（状态字低位 0x0250），按手册 4.1.1 该状态只接受 0x06，
  *         故第二次写 0x0006：既清掉 bit7，又停在「伺服准备好」，随后可直接调用 MID_LisanEc_EnableDrive
  *         继续 0x07、0x0F 完成状态机切换。若改写成 0x0007，其 bit0~bit3 构成的命令在「伺服无故障」
  *         下并非 4.1.1 列出的合法切换，属于未定义跳转。
  * @warning 本函数不复位过程耗时，若从站仍在「故障停机」过渡中，写入可能被忽略；
  *          需要严格确认时请先读 0x6041 确认已进入「伺服无故障」再调用。
  */
MID_CANopenStatusTypeDef MID_LisanEc_FaultReset(uint8_t nodeId)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;

  status = MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_FAULT_RESET);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_MOTOR_FREE);
}
#endif


/**
  * @brief  设置工作模式 0x6060。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  mode    工作模式，取 MID_LisanEcModeTypeDef 枚举值，取值参见 @ref MID_LISAN_EC_Operation_Mode。
  * @retval MID_CANOPEN_OK            设置成功。
  * @retval MID_CANOPEN_INVALID_PARAM 模式非法。
  * @retval 其他                      SDO 接口错误码。
  * @note   先校验模式属于轮廓位置、轮廓速度、力矩与回原点之一，非法直接返回；
  *         合法时把模式值以 8 位有符号写入 0x6060 子索引 0。
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
  * @note   指针为空判参数非法；读取 0x6061 的 8 位有符号值后强转为枚举输出。
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

#if (MID_LISAN_EC_ENABLE_DRIVE_PARAM == 1)
/**
  * @brief  设置电机细分 0x2001。
  * @param  nodeId         节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                        取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pulsePerRound  电机运行一圈所需脉冲数，写入 0x2001，由细分倍数与编码器线数共同决定。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  * @note   把每圈脉冲数直接写入 0x2001 子索引 0，不做数值校验。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMicroStep(uint8_t nodeId, uint32_t pulsePerRound)
{
  return MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_MICRO_STEP, MID_CANOPEN_OD_SUB_INDEX_0, pulsePerRound);
}
#endif


#if (MID_LISAN_EC_ENABLE_DRIVE_PARAM == 1)
/**
  * @brief  设置驱动器峰值电流 0x2000。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  currentMa  峰值电流，单位毫安，写入 0x2000，不得超过电机额定电流。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  * @note   把峰值电流直接写入 0x2000 子索引 0；不做上限校验，越界或超过额定值由从站以中止码拒绝。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetPeakCurrent(uint8_t nodeId, uint16_t currentMa)
{
  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_PEAK_CURRENT, MID_CANOPEN_OD_SUB_INDEX_0, currentMa);
}
#endif


#if (MID_LISAN_EC_ENABLE_DRIVE_PARAM == 1)
/**
  * @brief  设置平滑时间 0x2002。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  smoothTimeMs  平滑时间，单位毫秒，写入 0x2002，驱动器接受的范围为 1 至 1500。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  * @note   把平滑时间直接写入 0x2002 子索引 0；驱动器只接受 1 至 1500 毫秒，越界由从站以中止码拒绝。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetSmoothTime(uint8_t nodeId, uint16_t smoothTimeMs)
{
  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_SMOOTH_TIME, MID_CANOPEN_OD_SUB_INDEX_0, smoothTimeMs);
}
#endif


#if (MID_LISAN_EC_ENABLE_DRIVE_PARAM == 1)
/**
  * @brief  设置最小电流参数 0x2003。
  * @param  nodeId          节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                         取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  minimumCurrent  最小电流参数原始值，写入 0x2003；开环机型低 8 位为半流状态电流百分比，
  *                         闭环机型高低字节含义不同。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他            SDO 接口错误码。
  * @note   把参数原样写入 0x2003 子索引 0，字节含义随开环与闭环机型不同，不做换算与校验。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMinimumCurrent(uint8_t nodeId, uint16_t minimumCurrent)
{
  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_MINIMUM_CURRENT, MID_CANOPEN_OD_SUB_INDEX_0,
                                  minimumCurrent);
}
#endif


#if (MID_LISAN_EC_ENABLE_DRIVE_PARAM == 1)
/**
  * @brief  设置电机运行方向 0x2051。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  direction  电机运行方向，取 MID_LISAN_EC_DIRECTION_xxx，取值参见 @ref MID_LISAN_EC_Direction。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  * @note   把方向值写入 0x2051 子索引 0；不可在运行状态下修改。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMotorDirection(uint8_t nodeId, uint16_t direction)
{
  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_MOTOR_DIRECTION, MID_CANOPEN_OD_SUB_INDEX_0, direction);
}
#endif


/**
  * @brief  设置梯形加速度 0x6083。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  acceleration  梯形加速度，单位脉冲每平方秒，写入 0x6083。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  * @note   把梯形加速度直接写入 0x6083 子索引 0，驱动器据此换算加减速时间。
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
  * @note   把梯形减速度直接写入 0x6084 子索引 0，与加速度共同决定运行加减速斜坡。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetProfileDeceleration(uint8_t nodeId, uint32_t deceleration)
{
  return MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_PROFILE_DECELERATION, MID_CANOPEN_OD_SUB_INDEX_0,
                                  deceleration);
}

#if (MID_LISAN_EC_ENABLE_DRIVE_PARAM == 1)
/**
  * @brief  设置位置模式梯形速度 0x6081。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  velocity  位置模式下允许的最大速度，单位脉冲每秒，写入 0x6081，只有大小没有方向。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  * @note   把位置模式允许的最大速度写入 0x6081 子索引 0；该值只有大小，方向由目标位置决定。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetProfileVelocity(uint8_t nodeId, uint32_t velocity)
{
  return MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_PROFILE_VELOCITY, MID_CANOPEN_OD_SUB_INDEX_0, velocity);
}
#endif


/**
  * @brief  设置目标位置 0x607A。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  position  目标位置，单位脉冲，写入 0x607A。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  * @note   把目标位置写入 0x607A 子索引 0，单位脉冲。
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
  * @note   把目标速度写入 0x60FF 子索引 0，单位脉冲每秒，符号表示方向。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetTargetVelocity(uint8_t nodeId, int32_t velocity)
{
  return MID_CANopen_Sdo_WriteI32(nodeId, MID_CANOPEN_OD_TARGET_VELOCITY, MID_CANOPEN_OD_SUB_INDEX_0, velocity);
}

#if (MID_LISAN_EC_ENABLE_TORQUE_MODE == 1)
/**
  * @brief  设置目标力矩 0x6071。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  torque  力矩等级，写入 0x6071，数值表示大小，符号表示方向，取值参见 @ref MID_LISAN_EC_Torque_Range。
  * @retval MID_CANOPEN_OK            设置成功。
  * @retval MID_CANOPEN_INVALID_PARAM 力矩等级超出范围。
  * @retval 其他                      SDO 接口错误码。
  * @note   先校验力矩等级在 -255 至 255 之间，越界判参数非法；合法时写入 0x6071 子索引 0。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetTargetTorque(uint8_t nodeId, int16_t torque)
{
  if ((torque < MID_LISAN_EC_TORQUE_LEVEL_MIN) || (torque > MID_LISAN_EC_TORQUE_LEVEL_MAX))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_WriteI16(nodeId, MID_CANOPEN_OD_TARGET_TORQUE, MID_CANOPEN_OD_SUB_INDEX_0, torque);
}
#endif


#if (MID_LISAN_EC_ENABLE_TORQUE_MODE == 1)
/**
  * @brief  设置最大力矩 0x6072。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  maxTorque  最大力矩等级，写入 0x6072，数值越大可达到的最大电流越大，
  *                    取值参见 @ref MID_LISAN_EC_Torque_Range。
  * @retval MID_CANOPEN_OK            设置成功。
  * @retval MID_CANOPEN_INVALID_PARAM 力矩等级超出范围。
  * @retval 其他                      SDO 接口错误码。
  * @note   先校验最大力矩等级在 0 至 1000 之间，越界判参数非法；合法时写入 0x6072 子索引 0。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMaxTorque(uint8_t nodeId, uint16_t maxTorque)
{
  if ((maxTorque < MID_LISAN_EC_MAX_TORQUE_LEVEL_MIN) || (maxTorque > MID_LISAN_EC_MAX_TORQUE_LEVEL_MAX))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_WriteU16(nodeId, MID_CANOPEN_OD_MAX_TORQUE, MID_CANOPEN_OD_SUB_INDEX_0, maxTorque);
}
#endif


#if (MID_LISAN_EC_ENABLE_TORQUE_MODE == 1)
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
  * @note   先校验偏移脉冲不超过 16383；随后依次写入 0x2158 子索引 1 的工作方式、子索引 2 的偏移脉冲
  *         与子索引 3 的速度限制，任一步失败立即返回。
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
#endif


#if (MID_LISAN_EC_ENABLE_HOMING == 1)
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
  * @note   依次写入 0x6098 的回原点方式、0x6099 子索引 1 与 2 的快慢回原点速度、0x609A 的回原点加速度；
  *         任一步失败立即返回。
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
#endif


#if (MID_LISAN_EC_ENABLE_POSITION_MOVE == 1)
/**
  * @brief  以绝对位置方式启动轮廓位置模式运动。
  * @param  nodeId       节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                      取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  position     目标位置，单位脉冲，写入 0x607A。
  * @param  isImmediate  true 表示中断当前指令立即执行，false 表示等待当前指令执行完成后再生效。
  * @retval MID_CANOPEN_OK 指令已下发。
  * @retval 其他          SDO 接口错误码。
  * @note   先写目标位置 0x607A，再写 0x0F 基线控制字，最后把 bit4（新位置指令）置位后重写控制字；
  *         isImmediate 为 true 时同时置 bit5，中断当前指令立即执行新位置。
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
#endif


#if (MID_LISAN_EC_ENABLE_POSITION_MOVE == 1)
/**
  * @brief  以相对位置方式启动轮廓位置模式运动。
  * @param  nodeId         节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                        取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  deltaPosition  相对位移，单位脉冲，写入 0x607A，符号表示方向。
  * @param  isImmediate    true 表示中断当前指令立即执行，false 表示等待当前指令执行完成后再生效。
  * @retval MID_CANOPEN_OK 指令已下发。
  * @retval 其他          SDO 接口错误码。
  * @note   下发时序与绝对运动一致，区别是基线控制字额外置 bit6（相对位置），0x607A 按相对位移解释。
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
#endif


#if (MID_LISAN_EC_ENABLE_POSITION_MOVE == 1)
/**
  * @brief  清除新的位置指令。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  * @note   写回 0x0F 基线控制字，把 bit4 清零，撤销尚未生效的新位置指令。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ClearSetpoint(uint8_t nodeId)
{
  return MID_LisanEc_WriteControlWord(nodeId, MID_LISAN_EC_CONTROL_WORD_ENABLE_OPERATION);
}
#endif


#if (MID_LISAN_EC_ENABLE_HOMING == 1)
/**
  * @brief  启动回原点运动。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 指令已下发。
  * @retval 其他          SDO 接口错误码。
  * @note   依次写入控制字 0x07 锁机、0x0F 通电运行，最后写 0x1F（0x0F 叠加 bit4）触发一次新的位置指令。
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
#endif


/**
  * @brief  读取并解析状态字 0x6041。
  * @param  nodeId   节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                  取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pStatus  指向接收 0x6041 状态字解析结果的指针，取值参见 @ref MID_LISAN_EC_Status_Word，不可为 NULL。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 指针为空。
  * @retval 其他                      SDO 接口错误码。
  * @note   指针为空判参数非法；否则读取 0x6041 原始值后调用 MID_LisanEc_ParseStatusWord 填充结构体。
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
  * @note   指针为空判参数非法；否则读取 0x6064 的 32 位有符号实际位置，单位脉冲。
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
  * @note   指针为空判参数非法；否则读取 0x606C 的 32 位有符号实际速度，单位脉冲每秒。
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

#if (MID_LISAN_EC_ENABLE_MAINTENANCE == 1)
/**
  * @brief  读取实时电流 0x6078。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pCurrent  指向接收 0x6078 实时电流的变量的指针，单位毫安，不可为 NULL。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 指针为空。
  * @retval 其他                      SDO 接口错误码。
  * @note   指针为空判参数非法；否则读取 0x6078 的 16 位有符号实时电流，单位毫安。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadCurrent(uint8_t nodeId, int16_t *pCurrent)
{
  if (pCurrent == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_ReadI16(nodeId, MID_CANOPEN_OD_CURRENT_ACTUAL_VALUE, MID_CANOPEN_OD_SUB_INDEX_0, pCurrent);
}
#endif


/**
  * @brief  读取驱动器错误代码 0x603F。
  * @param  nodeId      节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                     取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pErrorCode  指向接收 0x603F 错误代码的变量的指针，取值参见 @ref MID_LISAN_EC_Error_Code，不可为 NULL。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 指针为空。
  * @retval 其他                      SDO 接口错误码。
  * @note   指针为空判参数非法；否则读取 0x603F 的最新错误代码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadErrorCode(uint8_t nodeId, uint16_t *pErrorCode)
{
  if (pErrorCode == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_ReadU16(nodeId, MID_CANOPEN_OD_ERROR_CODE, MID_CANOPEN_OD_SUB_INDEX_0, pErrorCode);
}

#if (MID_LISAN_EC_ENABLE_MAINTENANCE == 1)
/**
  * @brief  读取输入口与电机状态 0x60FD。
  * @param  nodeId   节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                  取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pInputs  指向接收 0x60FD 输入口与电机状态位图的变量的指针，bit0 至 bit7 对应 IN1 至 IN8，不可为 NULL。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 指针为空。
  * @retval 其他                      SDO 接口错误码。
  * @note   指针为空判参数非法；否则读取 0x60FD 的 32 位输入口与电机状态位图。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadDigitalInputs(uint8_t nodeId, uint32_t *pInputs)
{
  if (pInputs == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_Sdo_ReadU32(nodeId, MID_CANOPEN_OD_DIGITAL_INPUTS, MID_CANOPEN_OD_SUB_INDEX_0, pInputs);
}
#endif


#if (MID_LISAN_EC_ENABLE_MAINTENANCE == 1)
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
  * @note   先校验新地址落在 1 至 127；随后写入 0x2150 新地址，再按 isFromObject 写 0x2151 地址来源：
  *         true 为仅使用对象字典地址，false 为拨码开关地址与对象字典地址之和。
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
#endif


#if (MID_LISAN_EC_ENABLE_MAINTENANCE == 1)
/**
  * @brief  保存全部参数 0x1010 子索引 1。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 保存指令已发出。
  * @retval 其他          SDO 接口错误码。
  * @note   把保存签名写入 0x1010 子索引 1，驱动器收到后保存全部参数并重启。
  */
MID_CANopenStatusTypeDef MID_LisanEc_StoreParameters(uint8_t nodeId)
{
  return MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_STORE_PARAMETERS, MID_CANOPEN_OD_STORE_SUB_ALL,
                                  MID_CANOPEN_OD_STORE_SIGNATURE);
}
#endif


#if (MID_LISAN_EC_ENABLE_MAINTENANCE == 1)
/**
  * @brief  恢复全部出厂参数 0x1011 子索引 1。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 恢复指令已发出。
  * @retval 其他          SDO 接口错误码。
  * @note   把恢复签名写入 0x1011 子索引 1，驱动器恢复出厂参数并重启。
  */
MID_CANopenStatusTypeDef MID_LisanEc_RestoreDefault(uint8_t nodeId)
{
  return MID_CANopen_Sdo_WriteU32(nodeId, MID_CANOPEN_OD_RESTORE_DEFAULT, MID_CANOPEN_OD_RESTORE_SUB_ALL,
                                  MID_CANOPEN_OD_RESTORE_SIGNATURE);
}
#endif


/**
  * @brief  解析状态字得到 CiA402 驱动状态。
  * @param  statusWord  状态字 0x6041 原始数值，取值参见 @ref MID_LISAN_EC_Status_Word。
  * @retval 驱动状态。
  * @note   先按完整状态掩码逐项匹配 Operation Enabled、Quick Stop、Switched On 与 Ready to Switch On；
  *         再按低四位匹配未就绪与故障停机状态，全部不匹配时返回 UNKNOWN。
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
  * @note   按位展开各标志写入结构体：故障、使能、目标到达等为高有效，Quick Stop 为低有效，对应位为 0 时才置真。
  */
static void MID_LisanEc_ParseStatusWord(uint16_t statusWord, MID_LisanEcStatusWordTypeDef *pStatus)
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

#if (MID_LISAN_EC_ENABLE_MAINTENANCE == 1)
/**
  * @brief  将错误代码转换为可读文本。
  * @param  errorCode  错误代码 0x603F，取值参见 @ref MID_LISAN_EC_Error_Code。
  * @retval 错误文本，无故障返回 no error，未知代码返回 unknown。
  * @note   以错误代码为分支查表返回固定文本，只覆盖手册列举的几种故障。
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
#endif


/**
  * @brief  判断工作模式是否为驱动器支持的模式。
  * @param  mode  待判定的工作模式，取 MID_LisanEcModeTypeDef 枚举值，取值参见 @ref MID_LISAN_EC_Operation_Mode。
  * @retval true   驱动器支持该模式。
  * @retval false  驱动器不支持该模式。
  * @note   仅轮廓位置、轮廓速度、力矩与回原点四种模式返回 true，其余一律判为不支持。
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
