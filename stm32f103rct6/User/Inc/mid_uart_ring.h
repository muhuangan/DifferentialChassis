/**
  ******************************************************************************
  * @file    mid_uart_ring.h
  * @brief   UART4 循环 DMA 环形接收接口。
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
  * 本模块只负责把串口的 DMA 循环缓冲区当作环形缓冲区使用，对外提供零拷贝的读取接口，
  * 并在数据到达时通过注册回调通知上层。模块本身不依赖 RTOS，也不涉及 CAN。
  *
  ******************************************************************************
  */

#ifndef __MID_UART_RING_H
#define __MID_UART_RING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "chassis_link_config.h"

/**
  * @defgroup MID_UART_RING_Config 串口环形接收配置
  * @{
  */
#define MID_UART_RING_SIZE_BYTES  CHASSIS_LINK_RING_SIZE_BYTES  /**< 环形缓冲区容量，单位字节，取自 chassis_link_config.h。 */
#define MID_UART_RING_SIZE_MASK   (MID_UART_RING_SIZE_BYTES - 1U) /**< 环形缓冲区下标掩码。 */
/**
  * @}
  */

/**
  * @defgroup MID_UART_RING_Full_Threshold 环满判定阈值
  * @{
  */
#define MID_UART_RING_FULL_LEVEL  (MID_UART_RING_SIZE_BYTES - 1U) /**< 可用字节达到该值即认为环形缓冲区已被填满，消费者至少落后一整环。 */
/**
  * @}
  */

/**
  * @brief  数据到达通知回调。
  * @retval None
  * @warning 回调在中断上下文执行，只允许做置标志或释放信号量一类不阻塞的动作。
  */
typedef void (*MID_UartRingNotifyCallbackTypeDef)(void);

HAL_StatusTypeDef MID_UartRing_Init(void);
void MID_UartRing_SetNotifyCallback(MID_UartRingNotifyCallbackTypeDef callback);
void MID_UartRing_IdleIrqHandler(UART_HandleTypeDef *huart);
void MID_UartRing_DmaLapIrqHandler(DMA_HandleTypeDef *hdma);
uint16_t MID_UartRing_GetAvailable(void);
uint16_t MID_UartRing_Read(uint8_t *pDst, uint16_t length);
uint32_t MID_UartRing_GetOverflowCount(void);
uint32_t MID_UartRing_GetLapCount(void);
uint32_t MID_UartRing_GetReadBytes(void);

#if (CHASSIS_LINK_UPPER_SIM_ACTIVE == 1U)
/**
  * @brief  把环形缓冲区切到注入模式，不再由 UART4 的 DMA 写入。
  * @param  None
  * @retval HAL_OK     切换成功。
  * @note   写入下标改由软件维护，取数、解析与统计逻辑与 DMA 模式完全一致；
  *         上层因此可以像对待真实串口数据一样，从同一个环形缓冲区取数解析。
  *         本函数在调度器启动前调用，不会与 DMA 接收争用同一缓冲区。
  */
HAL_StatusTypeDef MID_UartRing_StartInject(void);

/**
  * @brief  向上层模拟一次串口数据到达，往环形缓冲区写入一段字节。
  * @param  pData   待写入数据首字节的指针，不可为 NULL。
  * @param  length  写入长度，单位字节。
  * @retval 实际写入字节数；未处于注入模式、指针为空或长度为 0 时返回 0。
  * @note   写入后像空闲中断一样触发一次数据到达通知，使等待信号量的任务被唤醒去取数，
  *         因此取数时序与真实串口保持一致；超出缓冲区容量的数据按环形回绕覆盖最旧内容。
  */
uint16_t MID_UartRing_Inject(const uint8_t *pData, uint16_t length);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MID_UART_RING_H */
