/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_chassis_link.h"
#include "chassis_link_config.h"
#if (CHASSIS_LINK_POWER_MONITOR_ENABLE == 1U)
#include "app_power_monitor.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
#if (CHASSIS_LINK_POWER_MONITOR_ENABLE == 1U)
/* 电源监视任务未登记在 .ioc 中，句柄与属性放在用户区，CubeMX 重新生成 freertos.c 时不会被覆盖。 */
osThreadId_t PowerMonitorHandle;
const osThreadAttr_t PowerMonitor_attributes = {
  .name = "PowerMonitor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
#endif
/* USER CODE END Variables */
/* Definitions for TaskLow */
osThreadId_t TaskLowHandle;
const osThreadAttr_t TaskLow_attributes = {
  .name = "TaskLow",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TaskHigh */
osThreadId_t TaskHighHandle;
const osThreadAttr_t TaskHigh_attributes = {
  .name = "TaskHigh",
  .stack_size = CHASSIS_LINK_TASK_STACK_BYTES,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskMid */
osThreadId_t TaskMidHandle;
const osThreadAttr_t TaskMid_attributes = {
  .name = "TaskMid",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
#if (CHASSIS_LINK_POWER_MONITOR_ENABLE == 1U)
void APP_PowerMonitor_Task(void *argument);
#endif
/* USER CODE END FunctionPrototypes */

void StartTaskLow(void *argument);
void StartTaskHigh(void *argument);
void StartTaskMid(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskLow */
  TaskLowHandle = osThreadNew(StartTaskLow, NULL, &TaskLow_attributes);

  /* creation of TaskHigh */
  TaskHighHandle = osThreadNew(StartTaskHigh, NULL, &TaskHigh_attributes);

  /* creation of TaskMid */
  TaskMidHandle = osThreadNew(StartTaskMid, NULL, &TaskMid_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
#if (CHASSIS_LINK_POWER_MONITOR_ENABLE == 1U)
  /* creation of PowerMonitor */
  PowerMonitorHandle = osThreadNew(APP_PowerMonitor_Task, NULL, &PowerMonitor_attributes);
#endif
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTaskLow */
/**
  * @brief  Function implementing the TaskLow thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskLow */
__weak void StartTaskLow(void *argument)
{
  /* USER CODE BEGIN StartTaskLow */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskLow */
}

/* USER CODE BEGIN Header_StartTaskHigh */
/**
* @brief Function implementing the TaskHigh thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskHigh */
__weak void StartTaskHigh(void *argument)
{
  /* USER CODE BEGIN StartTaskHigh */
  /* 底盘链路任务自带死循环，内部完成等待信号量、解析帧与转速下发。 */
  APP_ChassisLink_Task(argument);
  /* USER CODE END StartTaskHigh */
}

/* USER CODE BEGIN Header_StartTaskMid */
/**
* @brief Function implementing the TaskMid thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMid */
__weak void StartTaskMid(void *argument)
{
  /* USER CODE BEGIN StartTaskMid */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskMid */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

