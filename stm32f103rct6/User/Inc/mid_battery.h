/**
  ******************************************************************************
  * @file    mid_battery.h
  * @brief   电池电压采集，ADC 加 DMA 循环采样后换算毫伏与剩余电量。
  * @author  muhuangan
  * @date    2026-10-14
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
  * 采集链路为 ADC1 规则组第四通道经 DMA 循环写入四深度缓冲，本模块只取电池那一组。
  * 换算全程使用整数，避免在没有浮点单元的核上引入软浮点开销。
  *
  ******************************************************************************
  */

#ifndef __MID_BATTERY_H
#define __MID_BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
  * @brief  启动 ADC 校准并以 DMA 循环方式持续采样。
  * @param  None
  * @retval None
  * @note   F1 系列每次上电后必须先校准再启动转换，否则首批采样值偏大。
  *         调用前需确保 MX_ADC1_Init 已执行，且 ADC1 的 DMA 已由 MSP 绑定。
  */
void MID_Battery_Init(void);

/**
  * @brief  推进一次采样处理，更新电压、电量与低压状态。
  * @param  None
  * @retval None
  * @note   内部做滑动平均与状态消抖，调用周期由 CHASSIS_LINK_POWER_MONITOR_PERIOD_MS 决定，
  *         周期越短平均收敛越快，但不改变滤波深度。
  */
void MID_Battery_Refresh(void);

/**
  * @brief  读取当前电池电压。
  * @param  None
  * @retval 电压值，单位毫伏，未刷新前为满电估值。
  */
uint32_t MID_Battery_GetVoltageMv(void);

/**
  * @brief  读取当前剩余电量估计。
  * @param  None
  * @retval 电量百分比，取值 0 至 100。
  */
uint8_t MID_Battery_GetRemainPercent(void);

/**
  * @brief  判断电池是否进入低压告警。
  * @param  None
  * @retval true   电压持续低于低压门限并已消抖。
  * @retval false  电压未低于门限，或低压状态尚未消抖完成。
  */
bool MID_Battery_IsLow(void);

/**
  * @brief  判断电池是否进入即将截止的临界状态。
  * @param  None
  * @retval true   电压持续低于临界门限并已消抖。
  * @retval false  电压未低于门限，或临界状态尚未消抖完成。
  */
bool MID_Battery_IsCritical(void);

#ifdef __cplusplus
}
#endif

#endif /* __MID_BATTERY_H */
