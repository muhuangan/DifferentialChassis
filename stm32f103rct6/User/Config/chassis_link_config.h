/**
  ******************************************************************************
  * @file    chassis_link_config.h
  * @brief   底盘通信链路可调参数配置。
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

#ifndef __CHASSIS_LINK_CONFIG_H
#define __CHASSIS_LINK_CONFIG_H

/** @defgroup CHASSIS_LINK_Sync_Round 双轮速度模式一圈测试
 * @{
 */
#ifndef CHASSIS_LINK_SYNC_ROUND_ENABLE
#define CHASSIS_LINK_SYNC_ROUND_ENABLE 0U
#endif
#define CHASSIS_LINK_SYNC_ROUND_ACTIVE ((CHASSIS_LINK_SYNC_ROUND_ENABLE == 1U) && (CHASSIS_LINK_SELF_TEST_ENABLE == 1U) && !CHASSIS_LINK_POSITION_TEST_ACTIVE)
#define CHASSIS_LINK_GEAR_NUMERATOR 10U
#define CHASSIS_LINK_GEAR_DENOMINATOR 1U
#define CHASSIS_LINK_SYNC_WHEEL_COUNT 2U
#define CHASSIS_LINK_SYNC_MIN_SPEED_PPS 50U
#define CHASSIS_LINK_SYNC_POSITION_TOLERANCE_PULSES 20
#define CHASSIS_LINK_SYNC_SLOWDOWN_MS 2000U
#define CHASSIS_LINK_SYNC_STALL_MS 3000U
#define CHASSIS_LINK_SYNC_READY_TIMEOUT_MS 30000U
#define CHASSIS_LINK_SYNC_TIMEOUT_MARGIN_MS 15000U
/** @} */

/** @defgroup CHASSIS_LINK_Position_Test 有限位移测试配置
 * @{
 */
#ifndef CHASSIS_LINK_POSITION_TEST_ENABLE
#define CHASSIS_LINK_POSITION_TEST_ENABLE 0U
#endif
#define CHASSIS_LINK_POSITION_TEST_ACTIVE ((CHASSIS_LINK_POSITION_TEST_ENABLE == 1U) && (CHASSIS_LINK_SELF_TEST_ENABLE == 1U))
#define CHASSIS_LINK_POSITION_DISTANCE_PULSES 200
#define CHASSIS_LINK_POSITION_SPEED_PPS 500U
#define CHASSIS_LINK_POSITION_TOLERANCE_PULSES 10
#define CHASSIS_LINK_POSITION_TIMEOUT_MS 5000U
#define CHASSIS_LINK_POSITION_GAP_MS 1000U
/** @} */

/** @defgroup CHASSIS_LINK_Motor_Debug 启动诊断记录开关
 * @{
 */
#ifndef CHASSIS_LINK_MOTOR_DEBUG_ENABLE
#define CHASSIS_LINK_MOTOR_DEBUG_ENABLE 0U
#endif
/** @} */

/* 双轮同时速度测试：自检开启且未走位置测试时生效，与一圈测试、位置测试互斥。 */
#define CHASSIS_LINK_SPIN_TEST_ACTIVE ((CHASSIS_LINK_SELF_TEST_ENABLE == 1U) && !CHASSIS_LINK_POSITION_TEST_ACTIVE)

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @defgroup CHASSIS_LINK_Ring_Config 环形接收缓冲配置
  * @{
  */
#define CHASSIS_LINK_RING_SIZE_BYTES              512U  /**< 串口环形缓冲区容量，单位字节，必须为 2 的幂，取模由按位与实现。 */
#define CHASSIS_LINK_RING_SIZE_MASK               (CHASSIS_LINK_RING_SIZE_BYTES - 1U) /**< 环形缓冲区下标掩码。 */
#define CHASSIS_LINK_PARSE_CHUNK_BYTES            64U   /**< 单次排空环形缓冲区读取的字节数，单位字节，限制单次占用处理器的时间。 */
/**
  * @}
  */

/**
  * @defgroup CHASSIS_LINK_Task_Config 链路任务配置
  * @{
  */
#define CHASSIS_LINK_TASK_STACK_BYTES             1536U /**< 启动诊断与串口解析任务栈容量。 */
#define CHASSIS_LINK_POLL_TIMEOUT_MS              10U   /**< 等待接收信号量的超时，单位毫秒，超时即代表该时段无数据，同时兼作失联巡检周期。 */
#define CHASSIS_LINK_DRIVE_RETRY_PERIOD_MS        500U  /**< 驱动器初始化失败后的重试周期，单位毫秒。 */
#define CHASSIS_LINK_BOOT_WAIT_MS                 2500U /**< 等待驱动器首次心跳的时长。 */
#define CHASSIS_LINK_COMMAND_VERIFY_TIMEOUT_MS    2000U /**< 命令应答后等待读回生效的时长。 */
#define CHASSIS_LINK_MOTION_SPEED_TOLERANCE_PPS    20    /**< 实际速度与目标速度验证容差。 */
/**
  * @}
  */

/**
  * @defgroup CHASSIS_LINK_Wheel_Config 左右轮配置
  * @{
  */
#define CHASSIS_LINK_WHEEL_COUNT                  2U    /**< 轮子数量，用于遍历轮子参数表。 */
#define CHASSIS_LINK_WHEEL_INDEX_LEFT             0U    /**< 左轮在轮子参数表中的下标。 */
#define CHASSIS_LINK_WHEEL_INDEX_RIGHT            1U    /**< 右轮在轮子参数表中的下标。 */
#define CHASSIS_LINK_WHEEL_NODE_ID_LEFT           1U    /**< 左轮驱动器节点地址。 */
#define CHASSIS_LINK_WHEEL_NODE_ID_RIGHT          2U    /**< 右轮驱动器节点地址。 */
#define CHASSIS_LINK_WHEEL_NODE_ID_LIST           { CHASSIS_LINK_WHEEL_NODE_ID_LEFT, CHASSIS_LINK_WHEEL_NODE_ID_RIGHT } /**< 左右轮节点地址组，顺序为左轮在前、右轮在后，供节点注册与遍历统一取用。 */
/**
  * @}
  */

/**
  * @defgroup CHASSIS_LINK_SelfTest_Config 自检点动配置
  * @{
  */
/* 关闭 SYNC_ROUND 时执行双轮同时速度测试：地址组内全部节点同时起转，
   下限与上限取不同值时，目标速度按 CHASSIS_LINK_SPIN_RAMP_PPS_PER_MS 每毫秒一个脉冲在两者之间往复；
   下限与上限取相同值时目标速度恒定，不再爬坡。
   只写目标速度对象 0x60FF，不依赖位置反馈，轮子悬空与落地运行方式相同。
   开启 SYNC_ROUND 时恢复双轮同转一圈、按位置反馈停车的一圈测试。
   自检节点与轮节点取并集，总数不得超过 CANOPEN_MAX_NODE_COUNT。
   置 0 关闭全部上电自检，走上位机串口控制，此时本组参数不参与编译。 */
#define CHASSIS_LINK_SELF_TEST_ENABLE      0U   /**< 自检点动开关，1 开启；开启后上电即依次点动自检节点，不依赖上位机帧。 */
#define CHASSIS_LINK_SELF_TEST_NODE_ID_LIST { 1U, 2U } /**< 自检节点地址组，数组内节点同时转动，地址不得为 0。 */
#define CHASSIS_LINK_SELF_TEST_NODE_COUNT  2U   /**< 自检节点个数，必须与 CHASSIS_LINK_SELF_TEST_NODE_ID_LIST 元素个数一致。 */
#define CHASSIS_LINK_SELF_TEST_SPEED_PPS   2000 /**< 速度上限，单位脉冲每秒，正负表示方向。 */
#define CHASSIS_LINK_SPIN_MIN_SPEED_PPS    2000 /**< 速度下限，单位脉冲每秒；与上限取相同值即为恒定速度。 */
#define CHASSIS_LINK_SPIN_RAMP_PPS_PER_MS  1U   /**< 速度变化率，单位脉冲每秒每毫秒，1 即每毫秒变化一个脉冲。 */
#define CHASSIS_LINK_SPIN_UPDATE_MS        10U  /**< 目标速度下发周期，单位毫秒。 */
#define CHASSIS_LINK_SPIN_SAMPLE_MS        100U /**< 转动反馈采样周期，单位毫秒。 */
/**
  * @}
  */

/**
  * @defgroup CHASSIS_LINK_Speed_Config 目标转速配置
  * @{
  */
#define CHASSIS_LINK_SPEED_LIMIT_RPM              100    /**< 上位机帧内轮子转速绝对值上限，单位转每分，超出即判定帧非法并丢弃，需与上位机 rpm_limit 一致。 */
#define CHASSIS_LINK_SPEED_DEADBAND_PPS           10     /**< 转速变化死区，单位脉冲每秒，与上次下发值之差不超过该值时不重复下发。 */
#define CHASSIS_LINK_REFRESH_PERIOD_MS            100U   /**< 强制刷新周期，单位毫秒，转速未变也重发一次，防止偶发丢帧后长期停在旧值。 */
#define CHASSIS_LINK_SPEED_INVERT_LEFT            1U     /**< 左轮方向取反标志，1 取反，0 不取反；与上位机 left_direction 只能在一侧取反。 */
#define CHASSIS_LINK_SPEED_INVERT_RIGHT           0U     /**< 右轮方向取反标志，1 取反，0 不取反；与上位机 right_direction 只能在一侧取反。 */
#define CHASSIS_LINK_PULSES_PER_MOTOR_REV_DEFAULT 4000   /**< 2001h 读取失败时使用的电机每圈脉冲数默认值，需与驱动器细分一致。 */
/**
  * @}
  */

/**
  * @defgroup CHASSIS_LINK_Safety_Config 安全兜底配置
  * @{
  */
#define CHASSIS_LINK_COMM_TIMEOUT_MS              100U  /**< 通信超时，单位毫秒，超过该时长未收到合法帧即判定上位机失联。 */
#define CHASSIS_LINK_ENABLE_COMM_TIMEOUT_STOP     1U    /**< 失联自动停机开关，1 开启，0 关闭。 */
/**
  * @}
  */

/**
  * @defgroup CHASSIS_LINK_Ramp_Config 轮廓加减速配置
  * @{
  */
#define CHASSIS_LINK_ENABLE_PROFILE_RAMP          1U      /**< 1 处理加减速度，位置测试读取现有正值，速度模式写入配置值；0 跳过。 */
#define CHASSIS_LINK_PROFILE_ACCELERATION_PPS2    100000U  /**< 速度模式轮廓加速度，单位脉冲每平方秒，仅在开启写入时生效。 */
#define CHASSIS_LINK_PROFILE_DECELERATION_PPS2    200000U  /**< 轮廓减速度，单位脉冲每平方秒，仅在开启写入时生效，取值待实测确认。 */
/**
  * @}
  */

/**
  * @defgroup CHASSIS_LINK_Power_Config 电池电压与点阵屏配置
  * @{
  */
#ifndef CHASSIS_LINK_POWER_MONITOR_ENABLE
#define CHASSIS_LINK_POWER_MONITOR_ENABLE 1U
#endif
#define CHASSIS_LINK_BATTERY_ADC_INDEX            3U     /**< 电池通道在 ADC DMA 缓冲中的下标，对应规则组第四通道。 */
#define CHASSIS_LINK_BATTERY_DIVIDER_PERMILLE     11070  /**< 电池分压比，单位千分之一，11.07 对应 0 至 36 伏输入映射到 0 至 3.3 伏。 */
#define CHASSIS_LINK_BATTERY_CELL_COUNT           3U     /**< 电池串联串数，用于把总电压拆回单体电压查电量表。 */
#define CHASSIS_LINK_BATTERY_FULL_MV              12600  /**< 满电电压，单位毫伏，3 串锂电按单体 4.2 伏计。 */
#define CHASSIS_LINK_BATTERY_LOW_MV               10500  /**< 低压告警电压，单位毫伏，低于该值并持续消抖后置低压标志。 */
#define CHASSIS_LINK_BATTERY_CRITICAL_MV          9900   /**< 临界截止电压，单位毫伏，低于该值并持续消抖后置临界标志。 */
#define CHASSIS_LINK_POWER_MONITOR_PERIOD_MS      200U   /**< 电源监视任务周期，单位毫秒，同时决定采样与屏幕刷新速率。 */
/**
  * @}
  */

#if CHASSIS_LINK_SYNC_ROUND_ACTIVE
#if (CHASSIS_LINK_GEAR_NUMERATOR == 0U) || (CHASSIS_LINK_GEAR_DENOMINATOR == 0U) || \
    (CHASSIS_LINK_SELF_TEST_SPEED_PPS <= 0) || (CHASSIS_LINK_PROFILE_ACCELERATION_PPS2 == 0U) || \
    (CHASSIS_LINK_SYNC_SLOWDOWN_MS == 0U) || (CHASSIS_LINK_ENABLE_PROFILE_RAMP != 1U) || \
    (CHASSIS_LINK_SELF_TEST_NODE_COUNT != CHASSIS_LINK_SYNC_WHEEL_COUNT)
#error "Joint round test requires positive gear ratio, speed, acceleration and slowdown time."
#endif
#if (CHASSIS_LINK_PROFILE_DECELERATION_PPS2 * CHASSIS_LINK_SYNC_SLOWDOWN_MS) < \
    (CHASSIS_LINK_SELF_TEST_SPEED_PPS * 1000U)
#error "Slowdown time is too short for the configured speed and deceleration."
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_LINK_CONFIG_H */
