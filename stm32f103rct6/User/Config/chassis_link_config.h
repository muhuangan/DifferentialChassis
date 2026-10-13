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
#define CHASSIS_LINK_POLL_TIMEOUT_MS              10U   /**< 等待接收信号量的超时，单位毫秒，超时即代表该时段无数据，同时兼作失联巡检周期。 */
#define CHASSIS_LINK_DRIVE_RETRY_PERIOD_MS        500U  /**< 驱动器初始化失败后的重试周期，单位毫秒。 */
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
/* 用法：把 CHASSIS_LINK_SELF_TEST_ENABLE 置 1 后编译烧录，上电即按 CHASSIS_LINK_SELF_TEST_NODE_ID_LIST 的顺序依次点动：
   当前节点以 CHASSIS_LINK_SELF_TEST_SPEED_PPS 转动 CHASSIS_LINK_SELF_TEST_ON_MS，再停 CHASSIS_LINK_SELF_TEST_OFF_MS，
   随后轮到下一个节点，整个地址组走完从头循环。一次上电即可看清每个节点地址对应哪个轮子，同时验证全部驱动器的
   通信、使能与接线；确认后把开关改回 0 即恢复正常链路。
   注意自检节点会自动并入上电注册的节点列表（与 CHASSIS_LINK_WHEEL_NODE_ID_LIST 取并集），节点地址不得为 0，
   并集总数不得超过 CANOPEN_MAX_NODE_COUNT（默认 4），否则注册失败导致所有 SDO 超时。 */
#define CHASSIS_LINK_SELF_TEST_ENABLE      1U   /**< 自检点动开关，1 开启；开启后上电即依次点动自检节点，不依赖上位机帧。 */
#define CHASSIS_LINK_SELF_TEST_NODE_ID_LIST { 1U, 2U } /**< 自检节点地址组，按下标顺序依次点动，地址不得为 0。 */
#define CHASSIS_LINK_SELF_TEST_NODE_COUNT  2U   /**< 自检节点个数，必须与 CHASSIS_LINK_SELF_TEST_NODE_ID_LIST 元素个数一致。 */
#define CHASSIS_LINK_SELF_TEST_SPEED_PPS   5000 /**< 自检点动转速，单位脉冲每秒，符号表示方向。 */
#define CHASSIS_LINK_SELF_TEST_ON_MS       2000U /**< 单个自检节点的转动时长，单位毫秒。 */
#define CHASSIS_LINK_SELF_TEST_OFF_MS      1000U /**< 相邻两个自检节点之间的停止时长，单位毫秒。 */
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
#define CHASSIS_LINK_ENABLE_PROFILE_RAMP          1U      /**< 初始化阶段是否写入轮廓加减速度，1 写入，0 沿用驱动器出厂值。默认 1：手册 4.2.2 说明「一旦设置了速度、加速度和减速度，驱动器将根据加速度值驱动电机加速到运行速度」，不写则加减速沿用驱动器内部旧值，过小时表现为使能了但起不来。 */
#define CHASSIS_LINK_PROFILE_ACCELERATION_PPS2    10000U  /**< 轮廓加速度，单位脉冲每平方秒，仅在开启写入时生效。手册 PP/PV 示例用 2000 或 30000，此处取中间值。 */
#define CHASSIS_LINK_PROFILE_DECELERATION_PPS2    10000U  /**< 轮廓减速度，单位脉冲每平方秒，仅在开启写入时生效，取值待实测确认。 */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_LINK_CONFIG_H */
