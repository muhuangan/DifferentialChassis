/**
  ******************************************************************************
  * @file    mid_canopen.c
  * @brief   CANopen 主站协议栈实现。
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

#include "mid_canopen.h"

#include <string.h>

static MID_CANopenHandleTypeDef s_canopen;

static MID_CANopenStatusTypeDef MID_CANopen_SendFrame(uint32_t cobId, const uint8_t *pData, uint8_t length);
static MID_CANopenStatusTypeDef MID_CANopen_SdoExchange(uint8_t nodeId, const uint8_t *pRequest, uint8_t **ppResponse);
static MID_CANopenStatusTypeDef MID_CANopen_RecordAbort(uint8_t nodeId, const uint8_t *pResponse);
static MID_CANopenStatusTypeDef MID_CANopen_PdoConfigure(uint8_t nodeId, uint16_t communicationIndex,
                                                        uint16_t mappingIndex, const MID_CANopenPdoMapTypeDef *pMap,
                                                        uint8_t mapCount, uint32_t cobId, uint8_t transmissionType,
                                                        uint16_t eventTimerMs);
static uint8_t MID_CANopen_GetPdoOffset(MID_CANopenPdoIndexTypeDef pdoIndex);
static bool MID_CANopen_GetPdoIndex(uint16_t functionCode, bool isTpdo, MID_CANopenPdoIndexTypeDef *pPdoIndex);

/**
  * @brief  初始化 CANopen 协议栈。
  * @param  pHcan  指向已完成 MX_CAN_Init 初始化的 CAN 外设句柄的指针，单 CAN 工程通常传 &hcan，不可为 NULL。
  * @retval MID_CANOPEN_OK            初始化成功。
  * @retval MID_CANOPEN_INVALID_PARAM 句柄为空。
  * @retval MID_CANOPEN_ERROR         滤波器配置、启动或中断使能失败。
  * @note   内部完成全通滤波器配置、CAN 启动与接收中断使能。
  */
MID_CANopenStatusTypeDef MID_CANopen_Init(CAN_HandleTypeDef *pHcan)
{
  CAN_FilterTypeDef filterConfig = {0};

  if (pHcan == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  (void)memset(&s_canopen, 0, sizeof(s_canopen));
  s_canopen.pHcan = pHcan;

  filterConfig.FilterBank = 0U;
  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  filterConfig.FilterIdHigh = 0x0000U;
  filterConfig.FilterIdLow = 0x0000U;
  filterConfig.FilterMaskIdHigh = 0x0000U;
  filterConfig.FilterMaskIdLow = 0x0000U;
  filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filterConfig.FilterActivation = ENABLE;
  filterConfig.SlaveStartFilterBank = 14U;

  if (HAL_CAN_ConfigFilter(pHcan, &filterConfig) != HAL_OK)
  {
    return MID_CANOPEN_ERROR;
  }

  if (HAL_CAN_Start(pHcan) != HAL_OK)
  {
    return MID_CANOPEN_ERROR;
  }

  if (HAL_CAN_ActivateNotification(pHcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    return MID_CANOPEN_ERROR;
  }

  s_canopen.isReady = true;

  return MID_CANOPEN_OK;
}

/**
  * @brief  注册从站节点。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK            注册成功。
  * @retval MID_CANOPEN_INVALID_PARAM 节点地址非法或协议栈未初始化。
  * @retval MID_CANOPEN_ERROR         节点表已满或节点地址已注册。
  */
MID_CANopenStatusTypeDef MID_CANopen_AddNode(uint8_t nodeId)
{
  uint32_t index = 0U;

  if ((s_canopen.isReady == false) || (nodeId < MID_CANOPEN_NODE_ID_MIN) || (nodeId > MID_CANOPEN_NODE_ID_MAX))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  if (MID_CANopen_GetNode(nodeId) != NULL)
  {
    return MID_CANOPEN_ERROR;
  }

  for (index = 0U; index < CANOPEN_MAX_NODE_COUNT; index++)
  {
    if (s_canopen.nodes[index].isRegistered == false)
    {
      s_canopen.nodes[index].nodeId = nodeId;
      s_canopen.nodes[index].isRegistered = true;
      s_canopen.nodes[index].nmtState = MID_CANOPEN_NMT_STATE_UNKNOWN;
      return MID_CANOPEN_OK;
    }
  }

  return MID_CANOPEN_ERROR;
}

/**
  * @brief  查询节点运行信息。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval 节点信息指针。
  * @retval NULL    节点地址非法，或该地址未注册。
  * @note   本函数是库内按地址访问节点的唯一入口，SDO、PDO、TPDO 与心跳分发均经过它，
  *         因此节点地址的合法性校验集中在此处完成。
  */
MID_CANopenNodeTypeDef *MID_CANopen_GetNode(uint8_t nodeId)
{
  uint32_t index = 0U;

  /* 节点地址 0 是广播地址，大于 127 超出 CANopen 地址空间，二者都不是可访问的节点 */
  if ((nodeId < MID_CANOPEN_NODE_ID_MIN) || (nodeId > MID_CANOPEN_NODE_ID_MAX))
  {
    return NULL;
  }

  for (index = 0U; index < CANOPEN_MAX_NODE_COUNT; index++)
  {
    if ((s_canopen.nodes[index].isRegistered == true) && (s_canopen.nodes[index].nodeId == nodeId))
    {
      return &s_canopen.nodes[index];
    }
  }

  return NULL;
}

/**
  * @brief  判断节点心跳是否在线。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval true   节点在线。
  * @retval false  节点未注册或心跳超时。
  */
bool MID_CANopen_IsNodeOnline(uint8_t nodeId)
{
  MID_CANopenNodeTypeDef *pNode = MID_CANopen_GetNode(nodeId);

  if (pNode == NULL)
  {
    return false;
  }

  return pNode->isOnline;
}

/**
  * @brief  周期处理心跳超时判定。
  * @param  None
  * @retval None
  * @note   需在主循环或周期任务中调用。
  */
void MID_CANopen_Process(void)
{
  uint32_t index = 0U;

  for (index = 0U; index < CANOPEN_MAX_NODE_COUNT; index++)
  {
    MID_CANopenNodeTypeDef *pNode = &s_canopen.nodes[index];

    if (pNode->isRegistered == false)
    {
      continue;
    }

    if (pNode->hasHeartbeat == false)
    {
      pNode->isOnline = false;
      continue;
    }

    pNode->isOnline = ((HAL_GetTick() - pNode->heartbeatTick) <= CANOPEN_HEARTBEAT_TIMEOUT_MS) ? true : false;
  }
}

/**
  * @brief  发送网络管理报文。
  * @param  nodeId   目标节点地址，填 0 表示对总线上全部节点广播，填 1 至 127 表示仅控制单个节点，
  *                  取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  command  NMT 命令，取 MID_CANopenNmtCommandTypeDef 枚举值，取值参见 @ref MID_CANOPEN_NMT_Command。
  * @retval MID_CANOPEN_OK            发送成功。
  * @retval MID_CANOPEN_INVALID_PARAM 参数非法。
  * @retval MID_CANOPEN_TIMEOUT       发送邮箱等待超时。
  * @retval MID_CANOPEN_ERROR         发送失败。
  */
MID_CANopenStatusTypeDef MID_CANopen_Nmt_Control(uint8_t nodeId, MID_CANopenNmtCommandTypeDef command)
{
  uint8_t data[2] = {0};

  if ((nodeId > MID_CANOPEN_NODE_ID_MAX) || (s_canopen.isReady == false))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  data[0] = (uint8_t)command;
  data[1] = nodeId;

  return MID_CANopen_SendFrame(MID_CANOPEN_COB_NMT, data, 2U);
}

/**
  * @brief  发送同步帧。
  * @param  None
  * @retval MID_CANOPEN_OK      发送成功。
  * @retval MID_CANOPEN_TIMEOUT 发送邮箱等待超时。
  * @retval MID_CANOPEN_ERROR   发送失败。
  * @note   数据段长度为 0，用于触发同步类型 PDO 的收发。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sync_Send(void)
{
  static const uint8_t s_syncDummy = 0U;

  return MID_CANopen_SendFrame(MID_CANOPEN_COB_SYNC, &s_syncDummy, 0U);
}

/**
  * @brief  通过 SDO 加速传输写入对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  pData     指向待写入数据首字节的指针，函数按小端序取用其前 dataSize 个字节，不可为 NULL。
  * @param  dataSize  待写入字节数，取值范围 1 至 MID_CANOPEN_SDO_EXPEDITED_DATA_SIZE_BYTES（即 1 至 4），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant，必须与被写对象的字典长度一致，
  *                   否则从站以中止码 0x06070010 拒绝。
  * @retval MID_CANOPEN_OK            写入成功。
  * @retval MID_CANOPEN_INVALID_PARAM 参数非法。
  * @retval MID_CANOPEN_TIMEOUT       等待应答超时。
  * @retval MID_CANOPEN_ABORTED       从站返回中止码，可读取节点 lastSdoAbortCode。
  * @retval MID_CANOPEN_ERROR         应答异常或发送失败。
  * @note   驱动器对象字典中对象长度均不超过 4 字节，因此只实现加速传输。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_Write(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                                               const void *pData, uint8_t dataSize)
{
  uint8_t request[MID_CANOPEN_FRAME_DATA_SIZE_BYTES] = {0};
  uint8_t *pResponse = NULL;
  const uint8_t *pBytes = (const uint8_t *)pData;
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;
  uint8_t command = 0U;
  uint8_t byteIndex = 0U;

  if ((pData == NULL) || (dataSize == 0U) || (dataSize > MID_CANOPEN_SDO_EXPEDITED_DATA_SIZE_BYTES))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  request[0] = MID_CANOPEN_SDO_CCS_DOWNLOAD_EXPEDITED |
               (uint8_t)((MID_CANOPEN_SDO_EXPEDITED_DATA_SIZE_BYTES - dataSize) << 2U);
  request[1] = (uint8_t)(index & 0xFFU);
  request[2] = (uint8_t)(index >> 8U);
  request[3] = subIndex;

  for (byteIndex = 0U; byteIndex < dataSize; byteIndex++)
  {
    request[4U + byteIndex] = pBytes[byteIndex];
  }

  status = MID_CANopen_SdoExchange(nodeId, request, &pResponse);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  command = (uint8_t)(pResponse[0] & MID_CANOPEN_SDO_CCS_MASK);
  if (command == MID_CANOPEN_SDO_SCS_ABORT)
  {
    return MID_CANopen_RecordAbort(nodeId, pResponse);
  }

  if (command != MID_CANOPEN_SDO_SCS_DOWNLOAD_ACK)
  {
    return MID_CANOPEN_ERROR;
  }

  return MID_CANOPEN_OK;
}

/**
  * @brief  通过 SDO 加速传输读取对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  pData     接收读回数据的缓冲区指针，容量不小于 dataSize 个字节，不可为 NULL。
  * @param  dataSize  期望读回的字节数，取值范围 1 至 MID_CANOPEN_SDO_EXPEDITED_DATA_SIZE_BYTES（即 1 至 4），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant，必须与被读对象的字典长度一致，
  *                   否则返回 MID_CANOPEN_INVALID_PARAM。
  * @retval MID_CANOPEN_OK            读取成功。
  * @retval MID_CANOPEN_INVALID_PARAM 参数非法或返回长度与期望不符。
  * @retval MID_CANOPEN_TIMEOUT       等待应答超时。
  * @retval MID_CANOPEN_ABORTED       从站返回中止码，可读取节点 lastSdoAbortCode。
  * @retval MID_CANOPEN_ERROR         应答异常或发送失败。
  * @note   驱动器对象字典中对象长度均不超过 4 字节，因此只实现加速传输。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_Read(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                                              void *pData, uint8_t dataSize)
{
  uint8_t request[MID_CANOPEN_FRAME_DATA_SIZE_BYTES] = {0};
  uint8_t *pResponse = NULL;
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;
  uint8_t command = 0U;
  uint8_t responseSize = 0U;

  if ((pData == NULL) || (dataSize == 0U) || (dataSize > MID_CANOPEN_SDO_EXPEDITED_DATA_SIZE_BYTES))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  request[0] = MID_CANOPEN_SDO_CCS_UPLOAD_REQUEST;
  request[1] = (uint8_t)(index & 0xFFU);
  request[2] = (uint8_t)(index >> 8U);
  request[3] = subIndex;

  status = MID_CANopen_SdoExchange(nodeId, request, &pResponse);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  command = (uint8_t)(pResponse[0] & MID_CANOPEN_SDO_CCS_MASK);
  if (command == MID_CANOPEN_SDO_SCS_ABORT)
  {
    return MID_CANopen_RecordAbort(nodeId, pResponse);
  }

  if (command != MID_CANOPEN_SDO_SCS_UPLOAD_RESPONSE)
  {
    return MID_CANOPEN_ERROR;
  }

  responseSize = (uint8_t)(MID_CANOPEN_SDO_EXPEDITED_DATA_SIZE_BYTES -
                           ((pResponse[0] >> MID_CANOPEN_SDO_CMD_SHIFT) & MID_CANOPEN_SDO_SIZE_MASK));
  if (responseSize != dataSize)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  (void)memcpy(pData, &pResponse[4], (size_t)dataSize);

  return MID_CANOPEN_OK;
}

/**
  * @brief  以单字节写入对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  value     8 位无符号数值（0 至 255，对象字典类型 Uint8，按小端序写入 1 至 4 个字节）。
  * @retval MID_CANOPEN_OK 写入成功。
  * @retval 其他          MID_CANopen_Sdo_Write 的错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteU8(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t value)
{
  return MID_CANopen_Sdo_Write(nodeId, index, subIndex, &value, 1U);
}

/**
  * @brief  以双字节写入对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  value     16 位无符号数值（0 至 65535，对象字典类型 Uint16，按小端序写入 1 至 4 个字节）。
  * @retval MID_CANOPEN_OK 写入成功。
  * @retval 其他          MID_CANopen_Sdo_Write 的错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteU16(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint16_t value)
{
  return MID_CANopen_Sdo_Write(nodeId, index, subIndex, &value, 2U);
}

/**
  * @brief  以四字节写入对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  value     32 位无符号数值（0 至 4294967295，对象字典类型 Uint32，按小端序写入 1 至 4 个字节）。
  * @retval MID_CANOPEN_OK 写入成功。
  * @retval 其他          MID_CANopen_Sdo_Write 的错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteU32(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t value)
{
  return MID_CANopen_Sdo_Write(nodeId, index, subIndex, &value, 4U);
}

/**
  * @brief  以有符号单字节写入对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  value     8 位有符号数值（-128 至 127，对象字典类型 Int8，按小端序写入 1 至 4 个字节）。
  * @retval MID_CANOPEN_OK 写入成功。
  * @retval 其他          MID_CANopen_Sdo_Write 的错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteI8(uint8_t nodeId, uint16_t index, uint8_t subIndex, int8_t value)
{
  return MID_CANopen_Sdo_Write(nodeId, index, subIndex, &value, 1U);
}

/**
  * @brief  以有符号双字节写入对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  value     16 位有符号数值（-32768 至 32767，对象字典类型 Int16，按小端序写入 1 至 4 个字节）。
  * @retval MID_CANOPEN_OK 写入成功。
  * @retval 其他          MID_CANopen_Sdo_Write 的错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteI16(uint8_t nodeId, uint16_t index, uint8_t subIndex, int16_t value)
{
  return MID_CANopen_Sdo_Write(nodeId, index, subIndex, &value, 2U);
}

/**
  * @brief  以有符号四字节写入对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  value     32 位有符号数值（-2147483648 至 2147483647，对象字典类型 Int32，按小端序写入 1 至 4 个字节）。
  * @retval MID_CANOPEN_OK 写入成功。
  * @retval 其他          MID_CANopen_Sdo_Write 的错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteI32(uint8_t nodeId, uint16_t index, uint8_t subIndex, int32_t value)
{
  return MID_CANopen_Sdo_Write(nodeId, index, subIndex, &value, 4U);
}

/**
  * @brief  以单字节读取对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  pValue    指向接收读回8 位无符号数值的变量的指针，不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          MID_CANopen_Sdo_Read 的错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadU8(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t *pValue)
{
  return MID_CANopen_Sdo_Read(nodeId, index, subIndex, pValue, 1U);
}

/**
  * @brief  以双字节读取对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  pValue    指向接收读回16 位无符号数值的变量的指针，不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          MID_CANopen_Sdo_Read 的错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadU16(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint16_t *pValue)
{
  return MID_CANopen_Sdo_Read(nodeId, index, subIndex, pValue, 2U);
}

/**
  * @brief  以四字节读取对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  pValue    指向接收读回32 位无符号数值的变量的指针，不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          MID_CANopen_Sdo_Read 的错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadU32(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t *pValue)
{
  return MID_CANopen_Sdo_Read(nodeId, index, subIndex, pValue, 4U);
}

/**
  * @brief  以有符号单字节读取对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  pValue    指向接收读回8 位有符号数值的变量的指针，不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          MID_CANopen_Sdo_Read 的错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadI8(uint8_t nodeId, uint16_t index, uint8_t subIndex, int8_t *pValue)
{
  return MID_CANopen_Sdo_Read(nodeId, index, subIndex, pValue, 1U);
}

/**
  * @brief  以有符号双字节读取对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  pValue    指向接收读回16 位有符号数值的变量的指针，不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          MID_CANopen_Sdo_Read 的错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadI16(uint8_t nodeId, uint16_t index, uint8_t subIndex, int16_t *pValue)
{
  return MID_CANopen_Sdo_Read(nodeId, index, subIndex, pValue, 2U);
}

/**
  * @brief  以有符号四字节读取对象字典。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  index     对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                   @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex  对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                   @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  pValue    指向接收读回32 位有符号数值的变量的指针，不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          MID_CANopen_Sdo_Read 的错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadI32(uint8_t nodeId, uint16_t index, uint8_t subIndex, int32_t *pValue)
{
  return MID_CANopen_Sdo_Read(nodeId, index, subIndex, pValue, 4U);
}

/**
  * @brief  取 PDO 的默认 COB-ID。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pdoIndex  PDO 组号，取 MID_CANopenPdoIndexTypeDef 枚举值，取值参见 @ref MID_CANOPEN_PDO_Index。
  * @param  isTpdo    true 取 TPDO 的默认 COB-ID，false 取 RPDO 的默认 COB-ID。
  * @retval PDO 的默认 COB-ID，轴号或组号非法返回 0。
  */
uint32_t MID_CANopen_GetDefaultCobId(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex, bool isTpdo)
{
  uint16_t base = isTpdo ? MID_CANOPEN_COB_TPDO1_BASE : MID_CANOPEN_COB_RPDO1_BASE;
  uint16_t step = MID_CANOPEN_COB_TPDO2_BASE - MID_CANOPEN_COB_TPDO1_BASE;

  if ((nodeId < MID_CANOPEN_NODE_ID_MIN) || (nodeId > MID_CANOPEN_NODE_ID_MAX))
  {
    return 0U;
  }

  if ((pdoIndex < MID_CANOPEN_PDO_1) || (pdoIndex > MID_CANOPEN_PDO_4))
  {
    return 0U;
  }

  return (uint32_t)base + (uint32_t)(((uint16_t)pdoIndex - 1U) * step) + (uint32_t)nodeId;
}

/**
  * @brief  配置 RPDO 的通信参数与映射参数。
  * @param  nodeId            节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                           取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pdoIndex          PDO 组号，取 MID_CANopenPdoIndexTypeDef 枚举值，取值参见 @ref MID_CANOPEN_PDO_Index。
  * @param  pMap              指向映射项数组首元素的指针，数组元素个数由 mapCount 指定，
  *                           元素按 RPDO 数据段中字节出现的先后顺序排列，
  *                           其中 BitLength 取 MID_CANOPEN_OD_PDO_SIZE_xxx，
  *                           取值参见 @ref MID_CANOPEN_OD_PDO_Mapping_Entry，mapCount 非 0 时不可为 NULL。
  * @param  mapCount          映射项个数，取值范围 0 至 MID_CANOPEN_PDO_MAP_ITEM_MAX（即 0 至 8），
  *                           取值参见 @ref MID_CANOPEN_Protocol_Constant，填 0 表示清空该组映射。
  * @param  cobId             该组 PDO 使用的 11 位标准帧标识符（不含最高位无效标志），
  *                           通常取 MID_CANopen_GetDefaultCobId 的返回值，取值参见 @ref MID_CANOPEN_COB_ID_Base。
  * @param  transmissionType  PDO 传输类型，取值参见 @ref MID_CANOPEN_PDO_Transmission_Type。
  * @param  eventTimerMs      事件定时器周期，单位毫秒，填 0 表示不启用；RPDO 不使用该参数，任意取值均被忽略。
  * @retval MID_CANOPEN_OK            配置成功。
  * @retval MID_CANOPEN_INVALID_PARAM 参数非法或节点未注册。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Pdo_ConfigRx(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex,
                                                  const MID_CANopenPdoMapTypeDef *pMap, uint8_t mapCount,
                                                  uint32_t cobId, uint8_t transmissionType, uint16_t eventTimerMs)
{
  if (MID_CANopen_GetNode(nodeId) == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  if ((pdoIndex < MID_CANOPEN_PDO_1) || (pdoIndex > MID_CANOPEN_PDO_4))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  if ((mapCount > MID_CANOPEN_PDO_MAP_ITEM_MAX) || ((mapCount != 0U) && (pMap == NULL)))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_PdoConfigure(nodeId, (uint16_t)(MID_CANOPEN_OD_RPDO_COMM_PARAM + MID_CANopen_GetPdoOffset(pdoIndex)),
                                  (uint16_t)(MID_CANOPEN_OD_RPDO_MAPPING + MID_CANopen_GetPdoOffset(pdoIndex)),
                                  pMap, mapCount, cobId, transmissionType, eventTimerMs);
}

/**
  * @brief  配置 TPDO 的通信参数与映射参数。
  * @param  nodeId            节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                           取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pdoIndex          PDO 组号，取 MID_CANopenPdoIndexTypeDef 枚举值，取值参见 @ref MID_CANOPEN_PDO_Index。
  * @param  pMap              指向映射项数组首元素的指针，数组元素个数由 mapCount 指定，
  *                           元素按 TPDO 数据段中字节出现的先后顺序排列，
  *                           其中 BitLength 取 MID_CANOPEN_OD_PDO_SIZE_xxx，
  *                           取值参见 @ref MID_CANOPEN_OD_PDO_Mapping_Entry，mapCount 非 0 时不可为 NULL。
  * @param  mapCount          映射项个数，取值范围 0 至 MID_CANOPEN_PDO_MAP_ITEM_MAX（即 0 至 8），
  *                           取值参见 @ref MID_CANOPEN_Protocol_Constant，填 0 表示清空该组映射。
  * @param  cobId             该组 PDO 使用的 11 位标准帧标识符（不含最高位无效标志），
  *                           通常取 MID_CANopen_GetDefaultCobId 的返回值，取值参见 @ref MID_CANOPEN_COB_ID_Base。
  * @param  transmissionType  PDO 传输类型，取值参见 @ref MID_CANOPEN_PDO_Transmission_Type。
  * @param  eventTimerMs      事件定时器周期，单位毫秒，填 0 表示不启用；仅异步周期发送时有效。
  * @retval MID_CANOPEN_OK            配置成功。
  * @retval MID_CANOPEN_INVALID_PARAM 参数非法或节点未注册。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_CANopen_Pdo_ConfigTx(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex,
                                                  const MID_CANopenPdoMapTypeDef *pMap, uint8_t mapCount,
                                                  uint32_t cobId, uint8_t transmissionType, uint16_t eventTimerMs)
{
  if (MID_CANopen_GetNode(nodeId) == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  if ((pdoIndex < MID_CANOPEN_PDO_1) || (pdoIndex > MID_CANOPEN_PDO_4))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  if ((mapCount > MID_CANOPEN_PDO_MAP_ITEM_MAX) || ((mapCount != 0U) && (pMap == NULL)))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_PdoConfigure(nodeId, (uint16_t)(MID_CANOPEN_OD_TPDO_COMM_PARAM + MID_CANopen_GetPdoOffset(pdoIndex)),
                                  (uint16_t)(MID_CANOPEN_OD_TPDO_MAPPING + MID_CANopen_GetPdoOffset(pdoIndex)),
                                  pMap, mapCount, cobId, transmissionType, eventTimerMs);
}

/**
  * @brief  发送一帧 RPDO 数据。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pdoIndex  PDO 组号，取 MID_CANopenPdoIndexTypeDef 枚举值，取值参见 @ref MID_CANOPEN_PDO_Index。
  * @param  pData     指向待发送数据首字节的指针，函数发送其前 length 个字节，不可为 NULL。
  * @param  length    发送字节数，取值范围 0 至 MID_CANOPEN_FRAME_DATA_SIZE_BYTES（即 0 至 8），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant，应与该组 RPDO 映射的总长度一致。
  * @retval MID_CANOPEN_OK            发送成功。
  * @retval MID_CANOPEN_INVALID_PARAM 参数非法或节点未注册。
  * @retval MID_CANOPEN_TIMEOUT       发送邮箱等待超时。
  * @retval MID_CANOPEN_ERROR         发送失败。
  */
MID_CANopenStatusTypeDef MID_CANopen_Rpdo_Send(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex,
                                               const uint8_t *pData, uint8_t length)
{
  uint32_t cobId = 0U;

  if ((pData == NULL) || (MID_CANopen_GetNode(nodeId) == NULL) || (length > MID_CANOPEN_FRAME_DATA_SIZE_BYTES))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  cobId = MID_CANopen_GetDefaultCobId(nodeId, pdoIndex, false);
  if (cobId == 0U)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  return MID_CANopen_SendFrame(cobId, pData, length);
}

/**
  * @brief  读取最近一次接收到的 TPDO 数据。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pdoIndex  PDO 组号，取 MID_CANopenPdoIndexTypeDef 枚举值，取值参见 @ref MID_CANOPEN_PDO_Index。
  * @param  pData     接收 TPDO 数据的缓冲区指针，容量不小于 MID_CANOPEN_FRAME_DATA_SIZE_BYTES 个字节，
  *                   该长度定义于 @ref MID_CANOPEN_Protocol_Constant，不可为 NULL。
  * @param  pLength   指向接收本次数据长度的变量的指针，单位字节，不可为 NULL。
  * @retval true   读取成功。
  * @retval false  参数非法或该组 TPDO 尚无数据。
  */
bool MID_CANopen_GetTpdo(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex, uint8_t *pData, uint8_t *pLength)
{
  MID_CANopenNodeTypeDef *pNode = MID_CANopen_GetNode(nodeId);
  uint8_t pdoOffset = 0U;

  if ((pNode == NULL) || (pData == NULL) || (pLength == NULL))
  {
    return false;
  }

  if ((pdoIndex < MID_CANOPEN_PDO_1) || (pdoIndex > MID_CANOPEN_PDO_4))
  {
    return false;
  }

  pdoOffset = MID_CANopen_GetPdoOffset(pdoIndex);
  if (pNode->hasTpdo[pdoOffset] == false)
  {
    return false;
  }

  (void)memcpy(pData, pNode->tpdoBuffer[pdoOffset], pNode->tpdoLength[pdoOffset]);
  *pLength = pNode->tpdoLength[pdoOffset];

  return true;
}

/**
  * @brief  将 SDO 中止码转换为可读文本。
  * @param  abortCode  SDO 中止码，32 位，取值参见 @ref MID_CANOPEN_SDO_Abort_Code。
  * @retval 中止码文本，未知中止码返回 unknown。
  */
const char *MID_CANopen_SdoAbortToString(uint32_t abortCode)
{
  switch (abortCode)
  {
    case MID_CANOPEN_SDO_ABORT_TOGGLE_MISMATCH:
      return "toggle bit not altered";
    case MID_CANOPEN_SDO_ABORT_INVALID_COMMAND:
      return "invalid command specifier";
    case MID_CANOPEN_SDO_ABORT_UNSUPPORTED_ACCESS:
      return "unsupported access";
    case MID_CANOPEN_SDO_ABORT_READ_ONLY:
      return "object is read only";
    case MID_CANOPEN_SDO_ABORT_NOT_EXIST:
      return "object does not exist";
    case MID_CANOPEN_SDO_ABORT_DATA_MISMATCH:
      return "data type or length mismatch";
    case MID_CANOPEN_SDO_ABORT_OUT_OF_RANGE:
      return "value out of range";
    case MID_CANOPEN_SDO_ABORT_GENERAL:
      return "general error";
    default:
      return "unknown";
  }
}

/**
  * @brief  TPDO 数据到达回调。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pdoIndex  PDO 组号，取 MID_CANopenPdoIndexTypeDef 枚举值，取值参见 @ref MID_CANOPEN_PDO_Index。
  * @param  pData     指向本帧 TPDO 数据首字节的指针，指向节点内部缓存，回调返回后可能被下一帧覆盖，需要留存请自行拷贝。
  * @param  length    本帧数据长度，单位字节，最大值 MID_CANOPEN_FRAME_DATA_SIZE_BYTES，
  *                   该上限定义于 @ref MID_CANOPEN_Protocol_Constant。
  * @retval None
  * @note   默认实现为空，应用层可重定义以替换。
  */
__weak void MID_CANopen_TpdoReceivedCallback(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex,
                                             const uint8_t *pData, uint8_t length)
{
  (void)nodeId;
  (void)pdoIndex;
  (void)pData;
  (void)length;
}

/**
  * @brief  CAN 接收中断回调，分发 SDO、TPDO、紧急报文与心跳帧。
  * @param  pHcan  触发中断的 CAN 外设句柄。
  * @retval None
  * @note   中断中只做数据搬运与标志置位，不做阻塞操作。
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *pHcan)
{
  CAN_RxHeaderTypeDef rxHeader = {0};
  uint8_t data[MID_CANOPEN_FRAME_DATA_SIZE_BYTES] = {0};
  MID_CANopenNodeTypeDef *pNode = NULL;
  MID_CANopenPdoIndexTypeDef pdoIndex = MID_CANOPEN_PDO_1;
  uint16_t functionCode = 0U;
  uint8_t nodeId = 0U;
  uint8_t length = 0U;

  if ((s_canopen.isReady == false) || (pHcan != s_canopen.pHcan))
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(pHcan, CAN_FILTER_FIFO0, &rxHeader, data) != HAL_OK)
  {
    return;
  }

  if (rxHeader.IDE != CAN_ID_STD)
  {
    return;
  }

  if (rxHeader.StdId == MID_CANOPEN_COB_SYNC)
  {
    return;
  }

  functionCode = (uint16_t)(rxHeader.StdId & MID_CANOPEN_COB_ID_FUNCTION_MASK);
  nodeId = (uint8_t)(rxHeader.StdId & MID_CANOPEN_COB_ID_NODE_MASK);

  pNode = MID_CANopen_GetNode(nodeId);
  if (pNode == NULL)
  {
    return;
  }

  if (functionCode == MID_CANOPEN_COB_SDO_RESPONSE)
  {
    (void)memcpy(pNode->sdoResponse, data, MID_CANOPEN_FRAME_DATA_SIZE_BYTES);
    pNode->sdoResponseTick = HAL_GetTick();
    pNode->hasSdoResponse = true;
    return;
  }

  if (functionCode == MID_CANOPEN_COB_HEARTBEAT_BASE)
  {
    pNode->nmtState = (MID_CANopenNmtStateTypeDef)data[0];
    pNode->heartbeatTick = HAL_GetTick();
    pNode->heartbeatCount++;
    pNode->hasHeartbeat = true;
    pNode->isOnline = true;
    return;
  }

  if (functionCode == MID_CANOPEN_COB_EMCY_BASE)
  {
    pNode->lastEmcyCode = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
    pNode->emcyCount++;
    return;
  }

  if (MID_CANopen_GetPdoIndex(functionCode, true, &pdoIndex) == true)
  {
    uint8_t pdoOffset = MID_CANopen_GetPdoOffset(pdoIndex);

    length = (rxHeader.DLC > MID_CANOPEN_FRAME_DATA_SIZE_BYTES) ? MID_CANOPEN_FRAME_DATA_SIZE_BYTES
                                                              : (uint8_t)rxHeader.DLC;
    (void)memcpy(pNode->tpdoBuffer[pdoOffset], data, length);
    pNode->tpdoLength[pdoOffset] = length;
    pNode->tpdoTick[pdoOffset] = HAL_GetTick();
    pNode->hasTpdo[pdoOffset] = true;

    MID_CANopen_TpdoReceivedCallback(nodeId, pdoIndex, pNode->tpdoBuffer[pdoOffset], length);
  }
}

/**
  * @brief  向 CAN 总线发送一帧标准数据帧。
  * @param  cobId   11 位标准帧标识符，取值范围 0x000 至 MID_CANOPEN_COB_ID_MASK（即 0x7FF），
  *                 取值参见 @ref MID_CANOPEN_COB_ID_Base。
  * @param  pData   指向待发送数据首字节的指针，函数发送其前 length 个字节；length 为 0 时其内容不被读取，
  *                 但指针不可为 NULL。
  * @param  length  发送字节数，取值范围 0 至 MID_CANOPEN_FRAME_DATA_SIZE_BYTES（即 0 至 8），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK            发送成功。
  * @retval MID_CANOPEN_INVALID_PARAM 参数非法或协议栈未初始化。
  * @retval MID_CANOPEN_TIMEOUT       发送邮箱等待超时。
  * @retval MID_CANOPEN_ERROR         发送失败。
  */
static MID_CANopenStatusTypeDef MID_CANopen_SendFrame(uint32_t cobId, const uint8_t *pData, uint8_t length)
{
  CAN_TxHeaderTypeDef txHeader = {0};
  uint32_t mailbox = 0U;
  uint32_t startTick = 0U;

  if ((s_canopen.isReady == false) || (pData == NULL) || (length > MID_CANOPEN_FRAME_DATA_SIZE_BYTES))
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  txHeader.StdId = cobId & MID_CANOPEN_COB_ID_MASK;
  txHeader.ExtId = 0U;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = length;
  txHeader.TransmitGlobalTime = DISABLE;

  startTick = HAL_GetTick();
  while (HAL_CAN_GetTxMailboxesFreeLevel(s_canopen.pHcan) == 0U)
  {
    if ((HAL_GetTick() - startTick) > CANOPEN_TX_TIMEOUT_MS)
    {
      return MID_CANOPEN_TIMEOUT;
    }
  }

  if (HAL_CAN_AddTxMessage(s_canopen.pHcan, &txHeader, (uint8_t *)pData, &mailbox) != HAL_OK)
  {
    return MID_CANOPEN_ERROR;
  }

  return MID_CANOPEN_OK;
}

/**
  * @brief  发送 SDO 请求并等待从站应答。
  * @param  nodeId      节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                     取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pRequest    指向 8 字节 SDO 请求报文的指针，需在整个等待应答期间保持有效，不可为 NULL。
  * @param  ppResponse  指向“应答报文指针”的变量的指针，函数成功时被写入节点内部应答缓存的首地址；
  *                     填 NULL 表示不关心应答内容。
  * @retval MID_CANOPEN_OK            应答到达。
  * @retval MID_CANOPEN_INVALID_PARAM 节点未注册。
  * @retval MID_CANOPEN_TIMEOUT       等待应答超时。
  * @retval MID_CANOPEN_ERROR         发送失败。
  */
static MID_CANopenStatusTypeDef MID_CANopen_SdoExchange(uint8_t nodeId, const uint8_t *pRequest, uint8_t **ppResponse)
{
  MID_CANopenNodeTypeDef *pNode = MID_CANopen_GetNode(nodeId);
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;
  uint32_t startTick = 0U;

  if (pNode == NULL)
  {
    return MID_CANOPEN_INVALID_PARAM;
  }

  pNode->hasSdoResponse = false;

  status = MID_CANopen_SendFrame(MID_CANOPEN_COB_SDO_REQUEST + nodeId, pRequest, MID_CANOPEN_FRAME_DATA_SIZE_BYTES);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  startTick = HAL_GetTick();
  while (pNode->hasSdoResponse == false)
  {
    if ((HAL_GetTick() - startTick) > CANOPEN_SDO_TIMEOUT_MS)
    {
      return MID_CANOPEN_TIMEOUT;
    }
  }

  if (ppResponse != NULL)
  {
    *ppResponse = pNode->sdoResponse;
  }

  return MID_CANOPEN_OK;
}

/**
  * @brief  记录 SDO 中止码。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pResponse  指向从站返回的 8 字节中止报文的指针，不可为 NULL。
  * @retval MID_CANOPEN_ABORTED 固定返回，用于统一错误处理路径。
  */
static MID_CANopenStatusTypeDef MID_CANopen_RecordAbort(uint8_t nodeId, const uint8_t *pResponse)
{
  MID_CANopenNodeTypeDef *pNode = MID_CANopen_GetNode(nodeId);

  if (pNode != NULL)
  {
    pNode->lastSdoAbortCode = (uint32_t)pResponse[4] | ((uint32_t)pResponse[5] << 8U) |
                              ((uint32_t)pResponse[6] << 16U) | ((uint32_t)pResponse[7] << 24U);
  }

  return MID_CANOPEN_ABORTED;
}

/**
  * @brief  按手册 3.3.4 顺序写入 PDO 通信参数与映射参数。
  * @param  nodeId              节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                             取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  communicationIndex  PDO 通信参数索引，RPDO 为 MID_CANOPEN_OD_RPDO_COMM_PARAM+n，
  *                             TPDO 为 MID_CANOPEN_OD_TPDO_COMM_PARAM+n，n 为 0 至 3，
  *                             取值参见 @ref MID_CANOPEN_OD_Communication_Profile。
  * @param  mappingIndex        PDO 映射参数索引，RPDO 为 MID_CANOPEN_OD_RPDO_MAPPING+n，
  *                             TPDO 为 MID_CANOPEN_OD_TPDO_MAPPING+n，n 为 0 至 3，
  *                             取值参见 @ref MID_CANOPEN_OD_Communication_Profile。
  * @param  pMap                指向映射项数组首元素的指针，数组元素个数由 mapCount 指定，
  *                             其中 BitLength 取 MID_CANOPEN_OD_PDO_SIZE_xxx，
  *                             取值参见 @ref MID_CANOPEN_OD_PDO_Mapping_Entry，mapCount 非 0 时不可为 NULL。
  * @param  mapCount            映射项个数，取值范围 0 至 MID_CANOPEN_PDO_MAP_ITEM_MAX（即 0 至 8），
  *                             取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  cobId               该组 PDO 使用的 11 位标准帧标识符（不含最高位无效标志），
  *                             取值参见 @ref MID_CANOPEN_COB_ID_Base。
  * @param  transmissionType    PDO 传输类型，取值参见 @ref MID_CANOPEN_PDO_Transmission_Type。
  * @param  eventTimerMs        事件定时器周期，单位毫秒，填 0 表示不启用。
  * @retval MID_CANOPEN_OK 配置成功。
  * @retval 其他           SDO 接口错误码。
  */
static MID_CANopenStatusTypeDef MID_CANopen_PdoConfigure(uint8_t nodeId, uint16_t communicationIndex,
                                                         uint16_t mappingIndex, const MID_CANopenPdoMapTypeDef *pMap,
                                                         uint8_t mapCount, uint32_t cobId, uint8_t transmissionType,
                                                         uint16_t eventTimerMs)
{
  MID_CANopenStatusTypeDef status = MID_CANOPEN_OK;
  uint8_t index = 0U;

  status = MID_CANopen_Sdo_WriteU32(nodeId, communicationIndex, MID_CANOPEN_OD_PDO_SUB_COB_ID,
                                    cobId | MID_CANOPEN_COB_ID_INVALID_FLAG);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  status = MID_CANopen_Sdo_WriteU8(nodeId, mappingIndex, MID_CANOPEN_OD_PDO_SUB_MAP_COUNT, 0U);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  for (index = 0U; index < mapCount; index++)
  {
    uint32_t mapValue = MID_CANOPEN_OD_PDO_MAP_ENTRY(pMap[index].Index, pMap[index].SubIndex, pMap[index].BitLength);

    status = MID_CANopen_Sdo_WriteU32(nodeId, mappingIndex, (uint8_t)(MID_CANOPEN_OD_PDO_SUB_MAP_ITEM_1 + index),
                                      mapValue);
    if (status != MID_CANOPEN_OK)
    {
      return status;
    }
  }

  status = MID_CANopen_Sdo_WriteU8(nodeId, mappingIndex, MID_CANOPEN_OD_PDO_SUB_MAP_COUNT, mapCount);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  status = MID_CANopen_Sdo_WriteU8(nodeId, communicationIndex, MID_CANOPEN_OD_PDO_SUB_TRANSMISSION_TYPE,
                                   transmissionType);
  if (status != MID_CANOPEN_OK)
  {
    return status;
  }

  if (eventTimerMs != 0U)
  {
    status = MID_CANopen_Sdo_WriteU16(nodeId, communicationIndex, MID_CANOPEN_OD_PDO_SUB_EVENT_TIMER, eventTimerMs);
    if (status != MID_CANOPEN_OK)
    {
      return status;
    }
  }

  return MID_CANopen_Sdo_WriteU32(nodeId, communicationIndex, MID_CANOPEN_OD_PDO_SUB_COB_ID, cobId);
}

/**
  * @brief  取 PDO 组号对应的索引偏移。
  * @param  pdoIndex  PDO 组号，取 MID_CANopenPdoIndexTypeDef 枚举值，取值参见 @ref MID_CANOPEN_PDO_Index。
  * @retval 索引偏移，PDO1 返回 0，PDO4 返回 3，组号非法返回 0。
  */
static uint8_t MID_CANopen_GetPdoOffset(MID_CANopenPdoIndexTypeDef pdoIndex)
{
  if ((pdoIndex < MID_CANOPEN_PDO_1) || (pdoIndex > MID_CANOPEN_PDO_4))
  {
    return 0U;
  }

  return (uint8_t)((uint8_t)pdoIndex - 1U);
}

/**
  * @brief  按功能码解析 PDO 组号。
  * @param  functionCode  已按 MID_CANOPEN_COB_ID_FUNCTION_MASK 掩码后的 11 位标识符高 8 位功能码，
  *                       取值参见 @ref MID_CANOPEN_COB_ID_Base。
  * @param  isTpdo        true 按 TPDO 功能码解析，false 按 RPDO 功能码解析。
  * @param  pPdoIndex     指向接收解析出的 PDO 组号的变量的指针，仅在函数返回 true 时被写入，不可为 NULL。
  * @retval true   功能码为合法 PDO 功能码。
  * @retval false  功能码非 PDO。
  */
static bool MID_CANopen_GetPdoIndex(uint16_t functionCode, bool isTpdo, MID_CANopenPdoIndexTypeDef *pPdoIndex)
{
  uint16_t base = isTpdo ? MID_CANOPEN_COB_TPDO1_BASE : MID_CANOPEN_COB_RPDO1_BASE;
  uint16_t step = MID_CANOPEN_COB_TPDO2_BASE - MID_CANOPEN_COB_TPDO1_BASE;
  uint16_t last = (uint16_t)(base + ((MID_CANOPEN_PDO_COUNT - 1U) * step));

  if ((functionCode < base) || (functionCode > last) || (((functionCode - base) % step) != 0U))
  {
    return false;
  }

  *pPdoIndex = (MID_CANopenPdoIndexTypeDef)(((functionCode - base) / step) + 1U);

  return true;
}
