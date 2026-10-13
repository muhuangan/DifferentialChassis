/**
  ******************************************************************************
  * @file    app_chassis_link.h
  * @brief   底盘上位机链路应用层接口。
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
  * 本模块持有底盘链路的唯一任务与唯一信号量，把"排空环形缓冲、解析帧、下发转速"串成一条事件驱动链路。
  * 中断侧只做释放信号量一件事，剩余的解析与 CAN 下发全部在任务上下文完成。
  *
  ******************************************************************************
  */

#ifndef __APP_CHASSIS_LINK_H
#define __APP_CHASSIS_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "mid_canopen.h"

/**
  * @brief  底盘链路运行统计快照。
  */
typedef struct
{
  uint32_t validFrameCount;        /**< 合法帧累计数。 */
  uint32_t headerMismatchCount;    /**< 帧头失配累计数。 */
  uint32_t tailMismatchCount;      /**< 帧尾失配累计数。 */
  uint32_t rangeErrorCount;        /**< 范围校验失败累计数。 */
  uint32_t dispatchCount;          /**< 单轮转速下发成功累计数，一次双轮下发最多计两次。 */
  uint32_t dispatchFailureCount;   /**< 转速下发失败累计数。 */
  uint32_t lastSdoAbortCode;       /**< 最近一次下发失败对应节点的 SDO 中止码。 */
  uint32_t commLostCount;          /**< 通信超时触发停机的累计次数。 */
  uint32_t receiveLapCount;        /**< DMA 写满整环的圈数，长时间不增长说明串口无数据。 */
  uint32_t receiveOverflowCount;   /**< 环形缓冲区溢出累计次数，属环满指示而非精确丢字节数。 */
  uint32_t receiveByteCount;       /**< 已从环形缓冲区取走的总字节数。 */
  int32_t  targetLeftSpeed;        /**< 当前左轮目标速度，单位脉冲每秒。 */
  int32_t  targetRightSpeed;       /**< 当前右轮目标速度，单位脉冲每秒。 */
  int32_t  sentLeftSpeed;          /**< 最近一次下发的左轮速度，单位脉冲每秒。 */
  int32_t  sentRightSpeed;         /**< 最近一次下发的右轮速度，单位脉冲每秒。 */
  bool     isDriveReady;           /**< 两轮驱动器均已完成模式设置与使能。 */
  bool     isCommunicationLost;    /**< 上位机通信超时标志。 */
} APP_ChassisLinkStatisticsTypeDef;

MID_CANopenStatusTypeDef APP_ChassisLink_Init(void);
void APP_ChassisLink_RxNotifyFromIsr(void);
void APP_ChassisLink_Task(void *argument);
void APP_ChassisLink_GetStatistics(APP_ChassisLinkStatisticsTypeDef *pStatistics);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CHASSIS_LINK_H */
