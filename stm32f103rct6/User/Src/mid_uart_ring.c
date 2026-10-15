/**
  ******************************************************************************
  * @file    mid_uart_ring.c
  * @brief   UART4 循环 DMA 环形接收实现。
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

#include "mid_uart_ring.h"

#include <stddef.h>

#include "usart.h"
#include "dma.h"

extern DMA_HandleTypeDef hdma_uart4_rx;   /**< UART4 接收 DMA 句柄，由 MX_DMA_Init 创建。 */

static uint8_t s_ringBuffer[MID_UART_RING_SIZE_BYTES];            /**< DMA 循环写入的接收缓冲区。 */
static volatile uint16_t s_readPos = 0U;                          /**< 消费者读取下标。 */
static volatile uint32_t s_lapCount = 0U;                         /**< DMA 写满整环的圈数。 */
static volatile uint32_t s_overflowCount = 0U;                    /**< 环形缓冲区溢出累计次数。 */
static volatile uint32_t s_readBytes = 0U;                        /**< 已从环形缓冲区取走的总字节数。 */
static MID_UartRingNotifyCallbackTypeDef s_notifyCallback = NULL; /**< 数据到达通知回调，由上层注册。 */

#if (CHASSIS_LINK_UPPER_SIM_ACTIVE == 1U)
static volatile bool     s_isInjectMode = false;   /**< 是否处于注入模式；为真时写入下标不再取自 DMA 计数器。 */
static volatile uint16_t s_injectWritePos = 0U;    /**< 注入模式下的写入下标，环形下标。 */
#endif

static void MID_UartRing_Notify(void);

/**
  * @brief  触发一次数据到达通知。
  * @param  None
  * @retval None
  * @note   回调为空时静默返回；非空时直接调用注册的回调，因此本函数可能在中断上下文执行。
  */
static void MID_UartRing_Notify(void)
{
  if (s_notifyCallback != NULL)
  {
    s_notifyCallback();
  }
}

/**
  * @brief  启动 UART4 循环 DMA 接收并打开空闲中断。
  * @param  None
  * @retval HAL_OK     启动成功。
  * @retval HAL_ERROR  启动 DMA 接收失败。
  * @note   先把读取下标与圈数、溢出、已读字节三个统计量清零；
  *         再以 s_ringBuffer 为目标启动循环 DMA 接收；
  *         最后置位 UART_IT_IDLE，使总线静默时能通知上层取数。
  *         调用前需确保 MX_DMA_Init 与 MX_UART4_Init 已执行。
  */
HAL_StatusTypeDef MID_UartRing_Init(void)
{
  s_readPos = 0U;
  s_lapCount = 0U;
  s_overflowCount = 0U;
  s_readBytes = 0U;

  if (HAL_UART_Receive_DMA(&huart4, s_ringBuffer, MID_UART_RING_SIZE_BYTES) != HAL_OK)
  {
    return HAL_ERROR;
  }

  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);

  return HAL_OK;
}

/**
  * @brief  注册数据到达通知回调。
  * @param  callback  通知回调，填 NULL 表示取消通知。
  * @retval None
  * @note   仅保存函数指针，不触发任何通知；回调实际在中断上下文被调用。
  */
void MID_UartRing_SetNotifyCallback(MID_UartRingNotifyCallbackTypeDef callback)
{
  s_notifyCallback = callback;
}

/**
  * @brief  UART4 空闲中断处理，在总线静默时通知上层取数。
  * @param  huart  触发中断的串口句柄。
  * @retval None
  * @warning 仅限中断上下文调用，且必须在 HAL_UART_IRQHandler 之前执行。
  * @note   依次判断句柄非空且实例为 UART4、空闲标志与空闲中断使能均已置位，任一不满足即返回；
  *         条件都满足时先清空闲标志再触发一次通知。
  *         清标志需要先读 SR 再读 DR，此刻 RXNE 已由 DMA 取走，软件读 DR 不会抢走待搬运的字节。
  */
void MID_UartRing_IdleIrqHandler(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != UART4))
  {
    return;
  }

  if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == RESET) ||
      (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) == RESET))
  {
    return;
  }

  /* 清空闲标志需要先读 SR 再读 DR。空闲标志只在总线静默一个完整帧时间后置位，此刻 RXNE 已由 DMA 取走，
     软件读 DR 不会抢走 DMA 尚未搬运的字节。本钩子先于 HAL_UART_IRQHandler 执行，标志清掉后 HAL 不会再处理它。 */
  __HAL_UART_CLEAR_IDLEFLAG(huart);

  MID_UartRing_Notify();
}

/**
  * @brief  DMA 满一圈中断处理，作为空闲中断的兜底通知。
  * @param  hdma  触发中断的 DMA 句柄。
  * @retval None
  * @warning 仅限中断上下文调用，且必须在 HAL_DMA_IRQHandler 之前执行。
  * @note   判断句柄非空且实例为 UART4 接收通道后，再确认传输完成标志置位，否则直接返回；
  *         满足时圈计数加一并触发一次通知。
  *         此处不负责清标志，标志由随后的 HAL_DMA_IRQHandler 清除。
  */
void MID_UartRing_DmaLapIrqHandler(DMA_HandleTypeDef *hdma)
{
  if ((hdma == NULL) || (hdma->Instance != DMA2_Channel3))
  {
    return;
  }

  if (__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma)) == RESET)
  {
    return;
  }

  s_lapCount++;

  MID_UartRing_Notify();
}

/**
  * @brief  计算环形缓冲区中尚未取走的字节数。
  * @param  None
  * @retval 可读字节数。
  * @note   写入下标有两个来源：DMA 模式用剩余计数器反推，注入模式直接取软件维护的下标；
  *         两者与读取下标做差后按掩码回绕得到可读量，取数逻辑因此完全一致。
  *         DMA 模式下该值可能比真实值少 1 至 2 个字节，属保守方向，余量由下一次通知补齐。
  */
uint16_t MID_UartRing_GetAvailable(void)
{
  uint16_t writePos = 0U;

#if (CHASSIS_LINK_UPPER_SIM_ACTIVE == 1U)
  if (s_isInjectMode != false)
  {
    writePos = s_injectWritePos;
  }
  else
#endif
  {
    writePos = (uint16_t)((MID_UART_RING_SIZE_BYTES - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx)) &
                           MID_UART_RING_SIZE_MASK);
  }

  return (uint16_t)((writePos - s_readPos) & MID_UART_RING_SIZE_MASK);
}

/**
  * @brief  从环形缓冲区取走数据到调用方缓冲区。
  * @param  pDst    接收数据的缓冲区指针，不可为 NULL。
  * @param  length  期望取走的字节数，单位字节。
  * @retval 实际取走的字节数，等于期望字节数与当前可读字节数的较小值。
  * @note   入参非法时直接返回 0；否则先按当前可读量判断是否已环满并累加溢出计数，
  *         再逐字节搬到目标缓冲区并推进读取下标，最后把本次取走量累加到已读总字节数。
  */
uint16_t MID_UartRing_Read(uint8_t *pDst, uint16_t length)
{
  uint16_t available = 0U;
  uint16_t count = 0U;
  uint16_t index = 0U;

  if ((pDst == NULL) || (length == 0U))
  {
    return 0U;
  }

  available = MID_UartRing_GetAvailable();

  if (available >= MID_UART_RING_FULL_LEVEL)
  {
    s_overflowCount++;
  }

  count = (length < available) ? length : available;

  for (index = 0U; index < count; index++)
  {
    pDst[index] = s_ringBuffer[s_readPos];
    s_readPos = (uint16_t)((s_readPos + 1U) & MID_UART_RING_SIZE_MASK);
  }

  s_readBytes += count;

  return count;
}

/**
  * @brief  读取环形缓冲区溢出计数。
  * @param  None
  * @retval 溢出累计次数。
  * @note   计数在 MID_UartRing_Read 中可读量达到环满门限时累加，表示消费者落后整环的次数，
  *         属环满指示而非精确丢字节数。
  */
uint32_t MID_UartRing_GetOverflowCount(void)
{
  return s_overflowCount;
}

/**
  * @brief  读取 DMA 写满整环的圈数。
  * @param  None
  * @retval 圈数。
  * @note   每发生一次 DMA 传输完成中断加一；持续增长说明接收链路仍在工作，长时间不增长说明串口无数据。
  */
uint32_t MID_UartRing_GetLapCount(void)
{
  return s_lapCount;
}

#if (CHASSIS_LINK_UPPER_SIM_ACTIVE == 1U)
/**
  * @brief  把环形缓冲区切到注入模式，不再由 UART4 的 DMA 写入。
  * @param  None
  * @retval HAL_OK     切换成功。
  * @note   写入下标改由软件维护，取数、解析与统计逻辑与 DMA 模式完全一致；
  *         统计量与 DMA 模式一起清零，避免切换后残留上一次运行的计数。
  *         调用前需确保 MID_UartRing_Init 尚未启动 DMA 接收，否则两种写入方会争用同一缓冲区。
  */
HAL_StatusTypeDef MID_UartRing_StartInject(void)
{
  s_readPos = 0U;
  s_lapCount = 0U;
  s_overflowCount = 0U;
  s_readBytes = 0U;
  s_injectWritePos = 0U;
  s_isInjectMode = true;

  return HAL_OK;
}

/**
  * @brief  模拟一次串口数据到达，往环形缓冲区写入一段字节。
  * @param  pData   待写入数据首字节的指针，不可为 NULL。
  * @param  length  写入长度，单位字节。
  * @retval 实际写入字节数；未处于注入模式、指针为空或长度为 0 时返回 0。
  * @note   逐字节写入并按掩码推进下标，写满整环后回绕覆盖最旧内容；
  *         写完后触发一次数据到达通知，语义与空闲中断一致，
  *         使等待信号量的链路任务被唤醒并走与真实串口相同的取数路径。
  */
uint16_t MID_UartRing_Inject(const uint8_t *pData, uint16_t length)
{
  uint16_t index = 0U;

  if ((s_isInjectMode == false) || (pData == NULL) || (length == 0U))
  {
    return 0U;
  }

  for (index = 0U; index < length; index++)
  {
    s_ringBuffer[s_injectWritePos] = pData[index];
    s_injectWritePos = (uint16_t)((s_injectWritePos + 1U) & MID_UART_RING_SIZE_MASK);
  }

  MID_UartRing_Notify();

  return length;
}
#endif

/**
  * @brief  读取已从环形缓冲区取走的总字节数。
  * @param  None
  * @retval 总字节数。
  * @note   每次 MID_UartRing_Read 成功后按实际取走量累加，可用于估算上层处理速率。
  */
uint32_t MID_UartRing_GetReadBytes(void)
{
  return s_readBytes;
}
