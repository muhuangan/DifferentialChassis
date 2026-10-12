/**
  ******************************************************************************
  * @file    canopen_config.h
  * @brief   CANopen 通信库可调参数配置。
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

#ifndef __CANOPEN_CONFIG_H
#define __CANOPEN_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @defgroup CANOPEN_Node_Config CANopen Node Configuration
  * @{
  */
#define CANOPEN_MAX_NODE_COUNT            4U     /**< 总线可注册的从站数量上限。 */
#define CANOPEN_DEFAULT_NODE_ID           1U     /**< 从站出厂默认节点地址。 */
/**
  * @}
  */

/**
  * @defgroup CANOPEN_Timing_Config CANopen Timing Configuration
  * @{
  */
#define CANOPEN_TX_TIMEOUT_MS             10U    /**< 发送邮箱等待空闲的超时，单位毫秒。 */
#define CANOPEN_SDO_TIMEOUT_MS            100U   /**< 单次 SDO 请求等待应答的超时，单位毫秒。 */
#define CANOPEN_HEARTBEAT_TIMEOUT_MS      3000U  /**< 心跳帧丢失判定从站离线的超时，单位毫秒。 */
/**
  * @}
  */

/**
  * @defgroup CANOPEN_Debug_Config CANopen Debug Configuration
  * @{
  */
#define CANOPEN_IS_LOG_ENABLED            0U     /**< 日志开关，1 打开，0 关闭。 */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __CANOPEN_CONFIG_H */
