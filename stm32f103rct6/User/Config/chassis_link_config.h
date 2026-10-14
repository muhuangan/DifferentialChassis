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
#define CHASSIS_LINK_SYNC_ROUND_ENABLE 1U
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
/* 默认速度自检等待两轮就绪后同转一圈，按位置反馈减速停车。
   关闭 SYNC_ROUND 后执行逐轮正反各一次的速度测试。
   自检节点与轮节点取并集，总数不得超过 CANOPEN_MAX_NODE_COUNT。 */
#define CHASSIS_LINK_SELF_TEST_ENABLE      1U   /**< 自检点动开关，1 开启；开启后上电即依次点动自检节点，不依赖上位机帧。 */
#define CHASSIS_LINK_SELF_TEST_NODE_ID_LIST { 1U, 2U } /**< 自检节点地址组，按下标顺序依次点动，地址不得为 0。 */
#define CHASSIS_LINK_SELF_TEST_NODE_COUNT  2U   /**< 自检节点个数，必须与 CHASSIS_LINK_SELF_TEST_NODE_ID_LIST 元素个数一致。 */
#define CHASSIS_LINK_SELF_TEST_SPEED_PPS   2000 /**< 自检点动转速，单位脉冲每秒，符号表示方向。 */
#define CHASSIS_LINK_SELF_TEST_ON_MS       2000U /**< 每个方向到速确认后的保持时长，单位毫秒。 */
#define CHASSIS_LINK_SELF_TEST_OFF_MS      1000U /**< 停稳确认后换方向或节点的间隔，单位毫秒。 */
/**
  * @}
  */

/**
  * @defgroup CHASSIS_LINK_Speed_Config 目标转速配置
  * @{
  */
#define CHASSIS_LINK_SPEED_LIMIT_PPS              200000 /**< 目标转速绝对值上限，单位脉冲每秒，超出即判定帧非法并丢弃，取值待实测确认。 */
#define CHASSIS_LINK_SPEED_DEADBAND_PPS           10     /**< 转速变化死区，单位脉冲每秒，与上次下发值之差不超过该值时不重复下发。 */
#define CHASSIS_LINK_REFRESH_PERIOD_MS            100U   /**< 强制刷新周期，单位毫秒，转速未变也重发一次，防止偶发丢帧后长期停在旧值。 */
#define CHASSIS_LINK_SPEED_INVERT_LEFT            0U     /**< 左轮方向取反标志，1 取反，0 不取反。 */
#define CHASSIS_LINK_SPEED_INVERT_RIGHT           0U     /**< 右轮方向取反标志，1 取反，0 不取反。 */
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
#define CHASSIS_LINK_PROFILE_ACCELERATION_PPS2    1000U  /**< 速度模式轮廓加速度，单位脉冲每平方秒，仅在开启写入时生效。 */
#define CHASSIS_LINK_PROFILE_DECELERATION_PPS2    1000U  /**< 轮廓减速度，单位脉冲每平方秒，仅在开启写入时生效，取值待实测确认。 */
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
