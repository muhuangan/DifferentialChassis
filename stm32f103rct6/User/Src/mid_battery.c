/**
  ******************************************************************************
  * @file    mid_battery.c
  * @brief   电池电压采集实现，ADC 加 DMA 循环采样后换算毫伏与剩余电量。
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
  ******************************************************************************
  */

#include "mid_battery.h"

#include <stddef.h>
#include <string.h>

#include "main.h"
#include "adc.h"
#include "chassis_link_config.h"

#define MID_BATTERY_FILTER_DEPTH     8U   /**< 滑动平均深度，单位次，取 2 的幂便于移位。 */
#define MID_BATTERY_FULL_SCALE_MV    3300U /**< ADC 参考电压，单位毫伏。 */
#define MID_BATTERY_RAW_MAX          4095U /**< 12 位 ADC 满量程原始值。 */
#define MID_BATTERY_LOW_HOLD_MS      1000U /**< 低压判定消抖时长，单位毫秒。 */

/* 3 串锂电池单体电压到剩余电量的分段表，单位毫伏每串。
   顶端取 4180 而非 4200，是让 12 位 ADC 量化误差内的满电读数也能落到 100%，
   4.18 伏每串已进入充电截止平台区，按满电计不会高估可用容量。 */
static const uint16_t s_cellVoltageTableMv[] =
{
  3000U, 3450U, 3680U, 3740U, 3770U, 3790U, 3820U, 3870U, 3920U, 3980U, 4060U, 4180U
};

static const uint8_t s_cellPercentTable[] =
{
  0U, 5U, 10U, 20U, 30U, 40U, 50U, 60U, 70U, 80U, 90U, 100U
};

static volatile uint16_t s_rawBuffer[4U];       /**< ADC DMA 目标缓冲，四通道循环写入。 */
static uint16_t          s_filterSamples[MID_BATTERY_FILTER_DEPTH]; /**< 滑窗样本环形缓冲，单位原始码。 */
static uint8_t           s_filterNext = 0U;     /**< 下一个被替换的样本下标，滑窗满后循环推进。 */
static uint16_t          s_filterCount = 0U;    /**< 已进入滑窗的样本数，未满窗前按实际个数平均。 */
static uint32_t          s_filterSum = 0U;      /**< 滑窗样本累加值，单位原始码。 */
static uint16_t          s_averageRaw = 0U;     /**< 滑动平均后的原始码。 */
static uint32_t          s_voltageMv = 0U;      /**< 换算后的电池电压，单位毫伏。 */
static uint8_t           s_remainPercent = 0U;  /**< 剩余电量百分比。 */
static uint32_t          s_lowTick = 0U;        /**< 低压状态持续起始时刻，单位毫秒。 */
static bool              s_isLow = false;       /**< 低压告警标志。 */
static bool              s_isCritical = false;  /**< 临界截止标志。 */

static uint32_t MID_Battery_EstimatePercent(uint32_t voltageMv);

/**
  * @brief  由电池电压估计剩余电量。
  * @param  voltageMv 电池电压，单位毫伏。
  * @retval 电量百分比，取值 0 至 100。
  * @note   按串数把总电压拆回单体电压，再在分段表上线性插值；
  *         电压低于下限返回 0，高于上限返回 100。
  */
static uint32_t MID_Battery_EstimatePercent(uint32_t voltageMv)
{
  uint32_t cellMv = 0U;
  uint32_t index = 0U;
  uint32_t tableCount = (uint32_t)(sizeof(s_cellVoltageTableMv) / sizeof(s_cellVoltageTableMv[0]));

  cellMv = (voltageMv + (CHASSIS_LINK_BATTERY_CELL_COUNT / 2U)) / CHASSIS_LINK_BATTERY_CELL_COUNT;

  if (cellMv <= s_cellVoltageTableMv[0])
  {
    return 0U;
  }

  if (cellMv >= s_cellVoltageTableMv[tableCount - 1U])
  {
    return 100U;
  }

  while ((index + 1U) < tableCount)
  {
    if (cellMv < s_cellVoltageTableMv[index + 1U])
    {
      break;
    }

    index++;
  }

  {
    uint32_t lowMv = s_cellVoltageTableMv[index];
    uint32_t highMv = s_cellVoltageTableMv[index + 1U];
    uint32_t lowPercent = s_cellPercentTable[index];
    uint32_t highPercent = s_cellPercentTable[index + 1U];
    uint32_t span = highMv - lowMv;

    if (span == 0U)
    {
      return lowPercent;
    }

    return lowPercent + (((cellMv - lowMv) * (highPercent - lowPercent)) / span);
  }
}

void MID_Battery_Init(void)
{
  (void)HAL_ADCEx_Calibration_Start(&hadc1);
  (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_rawBuffer, 4U);

  (void)memset(s_filterSamples, 0, sizeof(s_filterSamples));
  s_filterNext = 0U;
  s_filterSum = 0U;
  s_filterCount = 0U;
  s_averageRaw = 0U;
  s_voltageMv = (uint32_t)CHASSIS_LINK_BATTERY_FULL_MV;
  s_remainPercent = 100U;
  s_lowTick = HAL_GetTick();
  s_isLow = false;
  s_isCritical = false;
}

void MID_Battery_Refresh(void)
{
  uint32_t raw = s_rawBuffer[CHASSIS_LINK_BATTERY_ADC_INDEX];
  uint32_t now = HAL_GetTick();
  uint32_t scaledMv = 0U;
  bool     isBelowLow = false;

  if (raw > MID_BATTERY_RAW_MAX)
  {
    raw = MID_BATTERY_RAW_MAX;
  }

  /* 入队新样本、出队最老样本；滑窗未满时按实际个数平均，避免开机初期被一个样本拉偏。
     阶跃输入后经过 MID_BATTERY_FILTER_DEPTH 次刷新即可完全收敛，不会留下长尾。 */
  s_filterSum += raw;
  if (s_filterCount < MID_BATTERY_FILTER_DEPTH)
  {
    s_filterSamples[s_filterCount] = (uint16_t)raw;
    s_filterCount++;
  }
  else
  {
    s_filterSum -= s_filterSamples[s_filterNext];
    s_filterSamples[s_filterNext] = (uint16_t)raw;
    s_filterNext = (uint8_t)((s_filterNext + 1U) % MID_BATTERY_FILTER_DEPTH);
  }

  s_averageRaw = (uint16_t)(s_filterSum / s_filterCount);

  /* 毫伏等于原始码乘参考电压除以满量程，再乘分压比；先乘后除并用长整型防溢出。 */
  scaledMv = (uint32_t)((((uint64_t)s_averageRaw * MID_BATTERY_FULL_SCALE_MV *
                          (uint64_t)CHASSIS_LINK_BATTERY_DIVIDER_PERMILLE) +
                         (uint64_t)(MID_BATTERY_RAW_MAX * 1000U / 2U)) /
                        (uint64_t)(MID_BATTERY_RAW_MAX * 1000U));

  s_voltageMv = scaledMv;
  s_remainPercent = (uint8_t)MID_Battery_EstimatePercent(scaledMv);

  /* 低压与临界共用一个起始时刻，电压回到门限以上立即解除，不附加迟滞以外的额外锁定。 */
  isBelowLow = (scaledMv < (uint32_t)CHASSIS_LINK_BATTERY_LOW_MV);
  if (isBelowLow == false)
  {
    s_lowTick = now;
    s_isLow = false;
    s_isCritical = false;
  }
  else if ((now - s_lowTick) >= MID_BATTERY_LOW_HOLD_MS)
  {
    s_isLow = true;
    s_isCritical = (scaledMv < (uint32_t)CHASSIS_LINK_BATTERY_CRITICAL_MV);
  }
}

uint32_t MID_Battery_GetVoltageMv(void)
{
  return s_voltageMv;
}

uint8_t MID_Battery_GetRemainPercent(void)
{
  return s_remainPercent;
}

bool MID_Battery_IsLow(void)
{
  return s_isLow;
}

bool MID_Battery_IsCritical(void)
{
  return s_isCritical;
}
