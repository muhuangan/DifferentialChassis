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
#if (MID_CANOPEN_ENABLE_PDO == 1)
#define MID_CANOPEN_COB_ID_INVALID_FLAG      0x80000000U /**< PDO COB-ID 无效标志，最高位为 1。 */
#endif

#define MID_CANOPEN_NODE_ID_MIN              1U      /**< 有效节点地址下限。 */
#define MID_CANOPEN_NODE_ID_MAX              127U    /**< 有效节点地址上限。 */
#define MID_CANOPEN_FRAME_DATA_SIZE_BYTES    8U      /**< 标准帧数据段长度，单位字节。 */
#define MID_CANOPEN_HEARTBEAT_DATA_SIZE_BYTES 1U     /**< 心跳报文数据段长度，单位字节。 */
#if (MID_CANOPEN_ENABLE_PDO == 1)
#define MID_CANOPEN_PDO_COUNT                4U      /**< 驱动器支持的 PDO 组数。 */
#endif

#if (MID_CANOPEN_ENABLE_PDO == 1)
#define MID_CANOPEN_PDO_MAP_ITEM_MAX         8U      /**< 单组 PDO 最多映射的对象个数。 */
#endif

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
#if (MID_CANOPEN_ENABLE_PDO == 1)
#define MID_CANOPEN_COB_TPDO1_BASE           0x180U  /**< TPDO1 功能码。 */
#define MID_CANOPEN_COB_RPDO1_BASE           0x200U  /**< RPDO1 功能码。 */
#define MID_CANOPEN_COB_TPDO2_BASE           0x280U  /**< TPDO2 功能码。 */
#define MID_CANOPEN_COB_RPDO2_BASE           0x300U  /**< RPDO2 功能码。 */
#define MID_CANOPEN_COB_TPDO3_BASE           0x380U  /**< TPDO3 功能码。 */
#define MID_CANOPEN_COB_RPDO3_BASE           0x400U  /**< RPDO3 功能码。 */
#define MID_CANOPEN_COB_TPDO4_BASE           0x480U  /**< TPDO4 功能码。 */
#define MID_CANOPEN_COB_RPDO4_BASE           0x500U  /**< RPDO4 功能码。 */
#endif

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
#define MID_CANOPEN_SDO_EXPEDITED_FLAGS      0x03U   /**< 加速传输与长度有效标志，e 为 bit1、s 为 bit0。 */
#define MID_CANOPEN_SDO_CCS_MASK             0xE0U   /**< 命令代码掩码。 */
#define MID_CANOPEN_SDO_CCS_DOWNLOAD_EXPEDITED 0x23U /**< 四字节加速下载请求，含 e=1、s=1；1、2、3 字节在其上按长度域叠加。 */
#define MID_CANOPEN_SDO_CCS_UPLOAD_REQUEST   0x40U   /**< 加速上传请求。 */
#define MID_CANOPEN_SDO_SCS_DOWNLOAD_ACK     0x60U   /**< 下载应答。 */
#define MID_CANOPEN_SDO_SCS_UPLOAD_RESPONSE  0x40U   /**< 上传应答。 */
#define MID_CANOPEN_SDO_SCS_ABORT            0x80U   /**< 中止传输。 */
#define MID_CANOPEN_SDO_CMD_SHIFT            2U      /**< 命令字中长度域右移位数。 */
#define MID_CANOPEN_SDO_SIZE_MASK            0x03U   /**< 命令字中长度域掩码。 */
/**
  * @}
  */

#if (MID_CANOPEN_ENABLE_PDO == 1)
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
#endif


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

#if (MID_CANOPEN_ENABLE_PDO == 1)
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
#endif


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
  volatile bool               isSdoWaiting;          /**< SDO 事务进行中标志，中断侧只接收当前事务的应答帧。 */
  volatile uint16_t           sdoExpectedIndex;      /**< 当前事务应答期望的对象索引。 */
  volatile uint8_t            sdoExpectedSubIndex;   /**< 当前事务应答期望的子索引。 */
  volatile uint8_t            sdoExpectedCommand;    /**< 当前事务应答期望上传或下载命令字。 */
  #if (MID_CANOPEN_ENABLE_PDO == 1)
bool                        hasTpdo[MID_CANOPEN_PDO_COUNT];   /**< 各 TPDO 数据到达标志，索引 0 对应 TPDO1。 */
  uint8_t                     tpdoLength[MID_CANOPEN_PDO_COUNT]; /**< 各 TPDO 数据长度，单位字节。 */
  uint8_t                     tpdoBuffer[MID_CANOPEN_PDO_COUNT][MID_CANOPEN_FRAME_DATA_SIZE_BYTES]; /**< 各 TPDO 数据缓存。 */
  uint32_t                    tpdoTick[MID_CANOPEN_PDO_COUNT];   /**< 各 TPDO 到达时间戳，单位毫秒。 */
#endif

} MID_CANopenNodeTypeDef;

/**
  * @brief  CANopen 协议栈实例。
  */
typedef struct
{
  CAN_HandleTypeDef   *hcan;                               /**< 绑定的 CAN 外设句柄。 */
  bool                isReady;                             /**< 协议栈就绪标志。 */
  MID_CANopenNodeTypeDef nodes[CANOPEN_MAX_NODE_COUNT];    /**< 已注册从站节点表。 */
} MID_CANopenHandleTypeDef;

/* CAN 总线诊断计数器：定义在 mid_canopen.c，刻意不声明为 static，供调试器与 live watch 直接读取，
   业务代码不应依赖其取值。排查「总线通不通、对端到底有哪些节点地址」时只看这三个变量。 */
extern volatile uint32_t g_canopenTxFrameCount;   /**< 累计成功写入发送邮箱的 CAN 帧数。 */
extern volatile uint32_t g_canopenRxFrameCount;   /**< 累计收到的 CAN 帧数，含未注册节点与已过滤的同步帧。 */
extern volatile uint32_t g_canopenLastRxCobId;    /**< 最近一帧的 11 位标准帧 ID，心跳为 0x700+NodeId。 */

MID_CANopenStatusTypeDef MID_CANopen_Init(CAN_HandleTypeDef *hcan);

MID_CANopenStatusTypeDef MID_CANopen_AddNode(uint8_t nodeId);

MID_CANopenNodeTypeDef *MID_CANopen_GetNode(uint8_t nodeId);

bool MID_CANopen_IsNodeOnline(uint8_t nodeId);

void MID_CANopen_Process(void);

MID_CANopenStatusTypeDef MID_CANopen_Nmt_Control(uint8_t nodeId, MID_CANopenNmtCommandTypeDef command);

#if (MID_CANOPEN_ENABLE_PDO == 1)
MID_CANopenStatusTypeDef MID_CANopen_Sync_Send(void);
#endif

MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteU8(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t value);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteU16(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint16_t value);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteU32(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t value);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteI8(uint8_t nodeId, uint16_t index, uint8_t subIndex, int8_t value);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteI16(uint8_t nodeId, uint16_t index, uint8_t subIndex, int16_t value);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_WriteI32(uint8_t nodeId, uint16_t index, uint8_t subIndex, int32_t value);

MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadU16(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint16_t *pValue);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadU32(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t *pValue);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadI8(uint8_t nodeId, uint16_t index, uint8_t subIndex, int8_t *pValue);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadI16(uint8_t nodeId, uint16_t index, uint8_t subIndex, int16_t *pValue);
MID_CANopenStatusTypeDef MID_CANopen_Sdo_ReadI32(uint8_t nodeId, uint16_t index, uint8_t subIndex, int32_t *pValue);

#if (MID_CANOPEN_ENABLE_PDO == 1)
uint32_t MID_CANopen_GetDefaultCobId(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex, bool isTpdo);
#endif

#if (MID_CANOPEN_ENABLE_PDO == 1)
MID_CANopenStatusTypeDef MID_CANopen_Pdo_ConfigRx(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex,
                                                  const MID_CANopenPdoMapTypeDef *pMap, uint8_t mapCount,
                                                  uint32_t cobId, uint8_t transmissionType, uint16_t eventTimerMs);
#endif

#if (MID_CANOPEN_ENABLE_PDO == 1)
MID_CANopenStatusTypeDef MID_CANopen_Pdo_ConfigTx(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex,
                                                  const MID_CANopenPdoMapTypeDef *pMap, uint8_t mapCount,
                                                  uint32_t cobId, uint8_t transmissionType, uint16_t eventTimerMs);
#endif

#if (MID_CANOPEN_ENABLE_PDO == 1)
MID_CANopenStatusTypeDef MID_CANopen_Rpdo_Send(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex,
                                               const uint8_t *pData, uint8_t length);
#endif

#if (MID_CANOPEN_ENABLE_PDO == 1)
bool MID_CANopen_GetTpdo(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex, uint8_t *pData, uint8_t *pLength);
#endif

const char *MID_CANopen_SdoAbortToString(uint32_t abortCode);

#if (MID_CANOPEN_ENABLE_PDO == 1)
void MID_CANopen_TpdoReceivedCallback(uint8_t nodeId, MID_CANopenPdoIndexTypeDef pdoIndex,
                                      const uint8_t *pData, uint8_t length);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MID_CANOPEN_H */
