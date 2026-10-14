/**
  ******************************************************************************
  * @file    app_power_monitor.h
  * @brief   电源监视应用，把电池电压与电量渲染到点阵屏。
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

#ifndef __APP_POWER_MONITOR_H
#define __APP_POWER_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  初始化电池采集与点阵屏。
  * @param  None
  * @retval None
  * @note   在调度器启动前调用，内部完成 ADC 校准启动与屏幕初始化。
  */
void APP_PowerMonitor_Init(void);

/**
  * @brief  电源监视任务主体，周期刷新采样并重绘屏幕。
  * @param  argument 未使用。
  * @retval None
  * @note   由 MX_FREERTOS_Init 创建；本任务只做采样与显示，不做控制决策。
  */
void APP_PowerMonitor_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __APP_POWER_MONITOR_H */
