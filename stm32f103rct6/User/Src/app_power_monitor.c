/**
  ******************************************************************************
  * @file    app_power_monitor.c
  * @brief   电源监视应用实现，把电池电压与电量渲染到点阵屏。
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
  * 屏幕分四页：首行电压，次行电量与进度条，三行状态，末行保留给后续信息。
  * 每轮先清帧缓冲再重绘，最后整帧推送，避免逐行写入造成可见闪烁。
  *
  ******************************************************************************
  */

#include "app_power_monitor.h"

#include <inttypes.h>
#include <stdio.h>

#include "cmsis_os.h"

#include "mid_battery.h"
#include "mid_oled.h"
#include "chassis_link_config.h"

#define APP_POWER_LINE_VOLTAGE       0U /**< 电压所在页。 */
#define APP_POWER_LINE_LEVEL         1U /**< 电量与进度条所在页。 */
#define APP_POWER_LINE_STATE         2U /**< 状态文字所在页。 */
#define APP_POWER_LINE_SPARE         3U /**< 预留页，当前留空。 */

#define APP_POWER_BAR_COLUMN         80U /**< 进度条起始列。 */
#define APP_POWER_BAR_WIDTH          44U /**< 进度条总宽，单位列。 */
#define APP_POWER_TEXT_LIMIT         24U /**< 状态文字缓冲长度，单位字节。 */

#define APP_POWER_OLED_SETTLE_MS     1000U /**< 上电后等待点阵屏内部复位与电荷泵稳定的时长，单位毫秒。 */

static void APP_PowerMonitor_Draw(uint32_t voltageMv, uint8_t percent);
static bool APP_PowerMonitor_IsChanged(uint32_t voltageMv, uint8_t percent);

static uint32_t s_lastShownVoltageMv = 0U; /**< 上一次已经推到屏幕的电压，单位毫伏。 */
static uint8_t  s_lastShownPercent = 0xFFU; /**< 上一次已经推到屏幕的电量，用无效值迫使首次刷新。 */

/**
  * @brief  判断显示内容是否与上一次不同。
  * @param  voltageMv 电池电压，单位毫伏。
  * @param  percent   电量百分比，取值 0 至 100。
  * @retval true   显示内容有变化，需要重绘并推送。
  * @retval false  显示内容未变，可跳过本次推送。
  * @note   电压按 10 毫伏取整后比较，避免采样噪声导致每轮都整帧刷新；
  *         一次整帧刷新要走完 512 字节软件 I2C，耗时约十几毫秒，能省则省。
  */
static bool APP_PowerMonitor_IsChanged(uint32_t voltageMv, uint8_t percent)
{
  bool isChanged = false;

  if ((voltageMv / 10U) != (s_lastShownVoltageMv / 10U))
  {
    isChanged = true;
  }

  if (percent != s_lastShownPercent)
  {
    isChanged = true;
  }

  if ((MID_Battery_IsLow() != false) || (MID_Battery_IsCritical() != false))
  {
    isChanged = true;
  }

  if (isChanged != false)
  {
    s_lastShownVoltageMv = voltageMv;
    s_lastShownPercent = percent;
  }

  return isChanged;
}

/**
  * @brief  绘制一整屏内容。
  * @param  voltageMv 电池电压，单位毫伏。
  * @param  percent   电量百分比，取值 0 至 100。
  * @retval None
  * @note   电压按两位小数显示，电量按整数百分比显示；状态行按低压与临界分别提示。
  */
static void APP_PowerMonitor_Draw(uint32_t voltageMv, uint8_t percent)
{
  char     text[APP_POWER_TEXT_LIMIT];
  uint32_t whole = voltageMv / 1000U;
  uint32_t fraction = (voltageMv % 1000U) / 10U;

  MID_Oled_Clear();

  (void)snprintf(text, sizeof(text), "BAT %" PRIu32 ".%02" PRIu32 "V", whole, fraction);
  MID_Oled_DrawString(0U, APP_POWER_LINE_VOLTAGE, text);

  (void)snprintf(text, sizeof(text), "%3u%%", percent);
  MID_Oled_DrawString(0U, APP_POWER_LINE_LEVEL, text);
  MID_Oled_DrawBar(APP_POWER_BAR_COLUMN, APP_POWER_LINE_LEVEL, APP_POWER_BAR_WIDTH,
                   (uint16_t)(percent * 10U));

  if (MID_Battery_IsCritical() != false)
  {
    (void)snprintf(text, sizeof(text), "SHUTDOWN NOW");
  }
  else if (MID_Battery_IsLow() != false)
  {
    (void)snprintf(text, sizeof(text), "LOW BATTERY");
  }
  else
  {
    (void)snprintf(text, sizeof(text), "READY");
  }

  MID_Oled_DrawString(0U, APP_POWER_LINE_STATE, text);

  MID_Oled_Flush();
}

void APP_PowerMonitor_Init(void)
{
  MID_Battery_Init();
}

void APP_PowerMonitor_Task(void *argument)
{
  (void)argument;

  /* 点阵屏上电后内部复位与电荷泵建立需要时间，初始化过早会一直黑屏（参考工程对此有明确备注，
     甚至在任务里初始化仍需上电复位才能显示）。因此先等系统起来一段时间，再初始化屏幕。 */
  (void)osDelay(APP_POWER_OLED_SETTLE_MS);

  MID_Oled_Init();

  for (;;)
  {
    MID_Battery_Refresh();

    if (APP_PowerMonitor_IsChanged(MID_Battery_GetVoltageMv(), MID_Battery_GetRemainPercent()) != false)
    {
      APP_PowerMonitor_Draw(MID_Battery_GetVoltageMv(), MID_Battery_GetRemainPercent());
    }

    (void)osDelay(CHASSIS_LINK_POWER_MONITOR_PERIOD_MS);
  }
}
