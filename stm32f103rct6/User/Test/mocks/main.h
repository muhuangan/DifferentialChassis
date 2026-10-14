/**
 ******************************************************************************
 * @file main.h
 * @brief 主机测试使用的最小 HAL 替身。
 * @author muhuangan
 * @attention Copyright 2026 muhuangan. MIT License; see project LICENSE.
 ******************************************************************************
 */
#ifndef __TEST_MAIN_H
#define __TEST_MAIN_H
#include <stdbool.h>
#include <stdint.h>
/** @defgroup TEST_HAL_Constants HAL 模拟常量
 * @{
 */
#define HAL_OK 0U
#define CAN_ESR_BOFF 4U
#define CAN_FILTER_FIFO0 0U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 1U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define CAN_ID_STD 0U
#define CAN_RTR_DATA 0U
#define ENABLE 1U
#define DISABLE 0U
#define __weak __attribute__((weak))
#define __DMB() __sync_synchronize()
/** @} */
typedef uint32_t HAL_StatusTypeDef;
typedef struct
{
  uint32_t ESR;
} CAN_TypeDef;
typedef struct
{
  CAN_TypeDef *Instance;
} CAN_HandleTypeDef;
typedef struct
{
  uint32_t StdId, ExtId, IDE, RTR, DLC, Timestamp, FilterMatchIndex;
} CAN_RxHeaderTypeDef;
typedef struct
{
  uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime;
} CAN_TxHeaderTypeDef;
typedef struct
{
  uint32_t FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow;
  uint32_t FilterFIFOAssignment, FilterBank, FilterMode, FilterScale;
  uint32_t FilterActivation, SlaveStartFilterBank;
} CAN_FilterTypeDef;
uint32_t          HAL_GetTick(void);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *pFilter);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t flags);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, uint32_t mailbox);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t *pData,
                                       uint32_t *pMailbox);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t fifo, CAN_RxHeaderTypeDef *pHeader,
                                       uint8_t *pData);
uint32_t          HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan);
void              HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
#endif
