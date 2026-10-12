/**
  ******************************************************************************
  * @file    mid_canopen.h
  * @brief   CANopen 主站协议栈接口，实现 NMT、SDO、PDO、同步与心跳对象。
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

#ifndef __MID_CANOPEN_H
#define __MID_CANOPEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "canopen_config.h"
#include "mid_canopen_od.h"

/**
  * @defgroup MID_CANOPEN_Protocol_Constant CANopen Protocol Constant
  * @{
  */
#define MID_CANOPEN_COB_ID_MASK              0x7FFU  /**< 11 位标准帧标识符掩码。 */
#define MID_CANOPEN_COB_ID_FUNCTION_MASK     0x780U  /**< 标识符中的功能码掩码。 */
#define MID_CANOPEN_COB_ID_NODE_MASK         0x07FU  /**< 标识符中的节点地址掩码。 */
#define MID_CANOPEN_COB_ID_INVALID_FLAG      0x80000000U /**< PDO COB-ID 无效标志，最高位为 1。 */
#define MID_CANOPEN_NODE_ID_MIN              1U      /**< 有效节点地址下限。 */
#define MID_CANOPEN_NODE_ID_MAX              127U    /**< 有效节点地址上限。 */
#define MID_CANOPEN_FRAME_DATA_SIZE_BYTES    8U      /**< 标准帧数据段长度，单位字节。 */
#define MID_CANOPEN_PDO_COUNT                4U      /**< 驱动器支持的 PDO 组数。 */
#define MID_CANOPEN_PDO_MAP_ITEM_MAX         8U      /**< 单组 PDO 最多映射的对象个数。 */
#define MID_CANOPEN_SDO_EXPEDITED_DATA_SIZE_BYTES 4U /**< 加速 SDO 可传输的最大字节数。 */
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_COB_ID_Base CANopen COB-ID Base
  * @{
  */
#define MID_CANOPEN_COB_NMT                  0x000U  /**< 网络管理报文功能码。 */
#define MID_CANOPEN_COB_SYNC                 0x080U  /**< 同步对象 COB-ID。 */
#define MID_CANOPEN_COB_EMCY_BASE            0x080U  /**< 紧急报文功能码。 */
#define MID_CANOPEN_COB_TPDO1_BASE           0x180U  /**< TPDO1 功能码。 */
#define MID_CANOPEN_COB_RPDO1_BASE           0x200U  /**< RPDO1 功能码。 */
#define MID_CANOPEN_COB_TPDO2_BASE           0x280U  /**< TPDO2 功能码。 */
#define MID_CANOPEN_COB_RPDO2_BASE           0x300U  /**< RPDO2 功能码。 */
#define MID_CANOPEN_COB_TPDO3_BASE           0x380U  /**< TPDO3 功能码。 */
#define MID_CANOPEN_COB_RPDO3_BASE           0x400U  /**< RPDO3 功能码。 */
#define MID_CANOPEN_COB_TPDO4_BASE           0x480U  /**< TPDO4 功能码。 */
#define MID_CANOPEN_COB_RPDO4_BASE           0x500U  /**< RPDO4 功能码。 */
#define MID_CANOPEN_COB_SDO_REQUEST          0x600U  /**< 主站发送 SDO 功能码。 */
#define MID_CANOPEN_COB_SDO_RESPONSE         0x580U  /**< 从站应答 SDO 功能码。 */
#define MID_CANOPEN_COB_HEARTBEAT_BASE       0x700U  /**< 心跳报文功能码。 */
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_SDO_Command CANopen SDO Command Specifier
  * @{
  */
#define MID_CANOPEN_SDO_CCS_MASK             0xE0U   /**< 命令代码掩码。 */
#define MID_CANOPEN_SDO_CCS_DOWNLOAD_EXPEDITED 0x20U /**< 加速下载请求。 */
#define MID_CANOPEN_SDO_CCS_UPLOAD_REQUEST   0x40U   /**< 加速上传请求。 */
#define MID_CANOPEN_SDO_SCS_DOWNLOAD_ACK     0x60U   /**< 下载应答。 */
#define MID_CANOPEN_SDO_SCS_UPLOAD_RESPONSE  0x40U   /**< 上传应答。 */
#define MID_CANOPEN_SDO_SCS_ABORT            0x80U   /**< 中止传输。 */
#define MID_CANOPEN_SDO_CMD_SHIFT            2U      /**< 命令字中长度域右移位数。 */
#define MID_CANOPEN_SDO_SIZE_MASK            0x03U   /**< 命令字中长度域掩码。 */
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_PDO_Transmission_Type CANopen PDO Transmission Type
  * @{
  */
#define MID_CANOPEN_PDO_TYPE_SYNC_ACYCLIC    0x00U   /**< 同步非循环，映射数据变化后由同步帧触发。 */
#define MID_CANOPEN_PDO_TYPE_SYNC_CYCLIC_1   0x01U   /**< 每接收到 1 个同步帧触发一次。 */
#define MID_CANOPEN_PDO_TYPE_ASYNC_EVENT     0xFEU   /**< 事件触发。 */
#define MID_CANOPEN_PDO_TYPE_ASYNC_VENDOR    0xFFU   /**< 厂商自定义事件触发。 */
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_SDO_Abort_Code CANopen SDO Abort Code
  * @{
  */
#define MID_CANOPEN_SDO_ABORT_INVALID_COMMAND    0x05040001U /**< 无效命令字。 */
#define MID_CANOPEN_SDO_ABORT_TOGGLE_MISMATCH    0x05030000U /**< 分段传输翻转位不匹配。 */
#define MID_CANOPEN_SDO_ABORT_UNSUPPORTED_ACCESS 0x06010000U /**< 不支持该访问方式。 */
#define MID_CANOPEN_SDO_ABORT_READ_ONLY          0x06010002U /**< 对象只读。 */
#define MID_CANOPEN_SDO_ABORT_NOT_EXIST          0x06020000U /**< 对象不存在。 */
#define MID_CANOPEN_SDO_ABORT_DATA_MISMATCH      0x06070010U /**< 数据类型或长度不匹配。 */
#define MID_CANOPEN_SDO_ABORT_OUT_OF_RANGE       0x06090030U /**< 数值超出对象取值范围。 */
#define MID_CANOPEN_SDO_ABORT_GENERAL            0x08000000U /**< 一般性错误。 */
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_Status CANopen Operation Status
  * @{
  */
typedef enum
{
  MID_CANOPEN_OK            = 0x00U, /**< 操作成功。 */
  MID_CANOPEN_ERROR         = 0x01U, /**< 一般错误。 */
  MID_CANOPEN_BUSY          = 0x02U, /**< 资源忙。 */
  MID_CANOPEN_TIMEOUT       = 0x03U, /**< 等待超时。 */
  MID_CANOPEN_INVALID_PARAM = 0x04U, /**< 参数非法。 */
  MID_CANOPEN_NOT_READY     = 0x05U, /**< 协议栈或节点未就绪。 */
  MID_CANOPEN_ABORTED       = 0x06U  /**< 从站以中止码拒绝了本次请求。 */
} MID_CANopenStatusTypeDef;
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_NMT_Command CANopen NMT Command
  * @{
  */
typedef enum
{
  MID_CANOPEN_NMT_START_REMOTE_NODE      = 0x01U, /**< 进入操作状态。 */
  MID_CANOPEN_NMT_STOP_REMOTE_NODE       = 0x02U, /**< 进入停止状态。 */
  MID_CANOPEN_NMT_ENTER_PRE_OPERATIONAL  = 0x80U, /**< 进入预操作状态。 */
  MID_CANOPEN_NMT_RESET_NODE             = 0x81U, /**< 复位节点。 */
  MID_CANOPEN_NMT_RESET_COMMUNICATION    = 0x82U  /**< 复位通信。 */
} MID_CANopenNmtCommandTypeDef;
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_NMT_State CANopen NMT State
  * @{
  */
typedef enum
{
  MID_CANOPEN_NMT_STATE_BOOT_UP          = 0x00U, /**< 上电初始化状态。 */
  MID_CANOPEN_NMT_STATE_STOPPED          = 0x04U, /**< 停止状态。 */
  MID_CANOPEN_NMT_STATE_OPERATIONAL      = 0x05U, /**< 操作状态。 */
  MID_CANOPEN_NMT_STATE_PRE_OPERATIONAL  = 0x7FU, /**< 预操作状态。 */
  MID_CANOPEN_NMT_STATE_UNKNOWN          = 0xFFU  /**< 尚未收到心跳帧。 */
} MID_CANopenNmtStateTypeDef;
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_PDO_Index CANopen PDO Index
  * @{
  */
typedef enum
{
  MID_CANOPEN_PDO_1 = 1U, /**< 第 1 组 PDO。 */
  MID_CANOPEN_PDO_2 = 2U, /**< 第 2 组 PDO。 */
  MID_CANOPEN_PDO_3 = 3U, /**< 第 3 组 PDO。 */
  MID_CANOPEN_PDO_4 = 4U  /**< 第 4 组 PDO。 */
} MID_CANopenPdoIndexTypeDef;
/**
  * @}
  */

/**
  * @brief  PDO 映射项。
  */
typedef struct
{
  uint16_t Index;     /**< 对象字典索引。 */
  uint8_t  SubIndex;  /**< 对象字典子索引。 */
  uint8_t  BitLength; /**< 对象长度，取 MID_CANOPEN_OD_PDO_SIZE_xxx。 */
} MID_CANopenPdoMapTypeDef;

/**
  * @brief  从站节点运行信息。
  */
typedef struct
{
  uint8_t                     nodeId;                /**< 节点地址。 */
  bool                        isRegistered;          /**< 节点已注册标志。 */
  bool                        isOnline;              /**< 心跳在线标志。 */
  bool                        hasHeartbeat;          /**< 是否收到过心跳帧。 */
  MID_CANopenNmtStateTypeDef  nmtState;              /**< 最近一次心跳上报的 NMT 状态。 */
  uint32_t                    heartbeatTick;         /**< 最近一次心跳帧时间戳，单位毫秒。 */
  uint32_t                    heartbeatCount;        /**< 心跳帧累计计数。 */
  uint16_t                    lastEmcyCode;          /**< 最近一次紧急报文的错误代码。 */
  uint32_t                    emcyCount;             /**< 紧急报文累计计数。 */
  uint32_t                    lastSdoAbortCode;      /**< 最近一次 SDO 中止码。 */
  bool                        hasSdoResponse;        /**< SDO 应答到达标志。 */
  uint32_t                    sdoResponseTick;       /**< SDO 应答到达时间戳，单位毫秒。 */
  uint8_t                     sdoResponse[8];        /**< SDO 应答原始数据。 */
  bool                        hasTpdo[MID_CANOPEN_PDO_COUNT];   /**< 各 TPDO 数据到达标志，索引 0 对应 TPDO1。 */
  uint8_t                     tpdoLength[MID_CANOPEN_PDO_COUNT]; /**< 各 TPDO 数据长度，单位字节。 */
  uint8_t                     tpdoBuffer[MID_CANOPEN_PDO_COUNT][MID_CANOPEN_FRAME_DATA_SIZE_BYTES]; /**< 各 TPDO 数据缓存。 */
  uint32_t                    tpdoTick[MID_CANOPEN_PDO_COUNT];   /**< 各 TPDO 到达时间戳，单位毫秒。 */
} MID_CANopenNodeTypeDef;

/**
  * @brief  CANopen 协议栈实例。
  */
typedef struct
{
  CAN_HandleTypeDef   *pHcan;                              /**< 绑定的 CAN 外设句柄。 */
  bool                isReady;                             /**< 协议栈就绪标志。 */
  MID_CANopenNodeTypeDef nodes[CANOPEN_MAX_NODE_COUNT];    /**< 已注册从站节点表。 */
} MID_CANopenHandleTypeDef;

/**
  * @brief  初始化 CANopen 协议栈。
  * @param  pHcan  指向已完成 MX_CAN_Init 初始化的 CAN 外设句柄的指针，单 CAN 工程通常传 &hcan，不可为 NULL。
  * @retval MID_CANOPEN_OK            初始化成功。
  * @retval MID_CANOPEN_INVALID_PARAM 句柄为空。
  * @retval MID_CANOPEN_ERROR         滤波器配置、启动或中断使能失败。
  * @note   内部完成全通滤波器配置、CAN 启动与接收中断使能。
  */
MID_CANopenStatusTypeDef MID_CANopen_Init(CAN_HandleTypeDef *pHcan);

/**
  * @brief  注册从站节点。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK            注册成功。
  * @retval MID_CANOPEN_INVALID_PARAM 节点地址非法或协议栈未初始化。
  * @retval MID_CANOPEN_ERROR         节点表已满或节点已注册。
  */
MID_CANopenStatusTypeDef MID_CANopen_AddNode(uint8_t nodeId);

/**
  * @brief  查询节点运行信息。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval 节点信息指针。
  * @retval NULL    节点地址非法，或该地址未注册。
  */
MID_CANopenNodeTypeDef *MID_CANopen_GetNode(uint8_t nodeId);

/**
  * @brief  判断节点心跳是否在线。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval true   节点在线。
  * @retval false  节点未注册或心跳超时。
  * @note   心跳超时判定由 MID_CANopen_Process 周期刷新。
  */
bool MID_CANopen_IsNodeOnline(uint8_t nodeId);

/**
  * @brief  周期处理心跳超时判定。
  * @param  None
  * @retval None
  * @note   需在主循环或周期任务中调用。
  */
void MID_CANopen_Process(void);

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
MID_CANopenStatusTypeDef MID_CANopen_Nmt_Control(uint8_t nodeId, MID_CANopenNmtCommandTypeDef command);

/**
  * @brief  发送同步帧。
  * @param  None
  * @retval MID_CANOPEN_OK      发送成功。
  * @retval MID_CANOPEN_TIMEOUT 发送邮箱等待超时。
  * @retval MID_CANOPEN_ERROR   发送失败。
  * @note   同步帧数据段长度为 0，用于触发同步类型 PDO 的收发。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sync_Send(void);

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
  * @note   该函数在等待应答期间阻塞，最长 CANOPEN_SDO_TIMEOUT_MS，禁止在中断中调用。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_Write(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                                               const void *pData, uint8_t dataSize);

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
  * @note   该函数在等待应答期间阻塞，最长 CANOPEN_SDO_TIMEOUT_MS，禁止在中断中调用。
  */
MID_CANopenStatusTypeDef MID_CANopen_Sdo_Read(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                                              void *pData, uint8_t dataSize);

MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteU8(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t value);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteU16(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint16_t value);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteU32(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t value);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteI8(uint8_t nodeId, uint16_t index, uint8_t subIndex, int8_t value);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteI16(uint8_t nodeId, uint16_t index, uint8_t subIndex, int16_t value);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteI32(uint8_t nodeId, uint16_t index, uint8_t subIndex, int32_t value);

MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadU8(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t *pValue);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadU16(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint16_t *pValue);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadU32(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t *pValue);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadI8(uint8_t nodeId, uint16_t index, uint8_t subIndex, int8_t *pValue);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadI16(uint8_t nodeId, uint16_t index, uint8_t subIndex, int16_t *pValue);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadI32(uint8_t nodeId, uint16_t index, uint8_t subIndex, int32_t *pValue);

/**
  * @brief  取 PDO 的默认 COB-ID。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pdoIndex  PDO 组号，取 MID_CANopenPdoIndexTypeDef 枚举值，取值参见 @ref MID_CANOPEN_PDO_Index。
  * @param  isTpdo    true 取 TPDO 的默认 COB-ID，false 取 RPDO 的默认 COB-ID。
  * @retval PDO 的默认 COB-ID，节点地址或组号非法返回 0。
  */
uint32_t MID_CANopen_GetDefaultCobId(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex, bool isTpdo);

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
  * @retval MID_CANOPEN_INVALID_PARAM 参数非法。
  * @retval MID_CANOPEN_TIMEOUT       等待应答超时。
  * @retval MID_CANOPEN_ABORTED       从站返回中止码。
  * @retval MID_CANOPEN_ERROR         应答异常或发送失败。
  * @note   配置顺序遵循手册 3.3.4 示例，先禁用 PDO 再改写映射，最后激活。
  */
MID_CANopenStatusTypeDef MID_CANopen_Pdo_ConfigRx(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex,
                                                  const MID_CANopenPdoMapTypeDef *pMap, uint8_t mapCount,
                                                  uint32_t cobId, uint8_t transmissionType, uint16_t eventTimerMs);

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
  * @retval MID_CANOPEN_INVALID_PARAM 参数非法。
  * @retval MID_CANOPEN_TIMEOUT       等待应答超时。
  * @retval MID_CANOPEN_ABORTED       从站返回中止码。
  * @retval MID_CANOPEN_ERROR         应答异常或发送失败。
  * @note   自动传输方向下的 TPDO 为异步周期发送时，需保证节点处于操作状态。
  */
MID_CANopenStatusTypeDef MID_CANopen_Pdo_ConfigTx(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex,
                                                  const MID_CANopenPdoMapTypeDef *pMap, uint8_t mapCount,
                                                  uint32_t cobId, uint8_t transmissionType, uint16_t eventTimerMs);

/**
  * @brief  发送一帧 RPDO 数据。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pdoIndex  PDO 组号，取 MID_CANopenPdoIndexTypeDef 枚举值，取值参见 @ref MID_CANOPEN_PDO_Index。
  * @param  pData     指向待发送数据首字节的指针，函数发送其前 length 个字节，不可为 NULL。
  * @param  length    发送字节数，取值范围 0 至 MID_CANOPEN_FRAME_DATA_SIZE_BYTES（即 0 至 8），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant，应与该组 RPDO 映射的总长度一致。
  * @retval MID_CANOPEN_OK            发送成功。
  * @retval MID_CANOPEN_INVALID_PARAM 参数非法。
  * @retval MID_CANOPEN_TIMEOUT       发送邮箱等待超时。
  * @retval MID_CANOPEN_ERROR         发送失败。
  */
MID_CANopenStatusTypeDef MID_CANopen_Rpdo_Send(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex,
                                               const uint8_t *pData, uint8_t length);

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
bool MID_CANopen_GetTpdo(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex, uint8_t *pData, uint8_t *pLength);

/**
  * @brief  将 SDO 中止码转换为可读文本。
  * @param  abortCode  SDO 中止码，32 位，取值参见 @ref MID_CANOPEN_SDO_Abort_Code。
  * @retval 中止码文本，未知中止码返回 unknown。
  */
const char *MID_CANopen_SdoAbortToString(uint32_t abortCode);

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
void MID_CANopen_TpdoReceivedCallback(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex,
                                      const uint8_t *pData, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* __MID_CANOPEN_H */
