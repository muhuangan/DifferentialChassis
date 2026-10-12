/**
  ******************************************************************************
  * @file    mid_canopen_od.h
  * @brief   CANopen 对象字典索引与子索引定义。
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
  * 索引取值来源立三 EC 系列步进驱动器 CANopen 通讯手册 V4.02 第六章参数设置一览表。
  *
  ******************************************************************************
  */

#ifndef __MID_CANOPEN_OD_H
#define __MID_CANOPEN_OD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @defgroup MID_CANOPEN_OD_Communication_Profile CANopen Communication Profile Area
  * @{
  */
#define MID_CANOPEN_OD_DEVICE_TYPE               0x1000U /**< 设备类型。 */
#define MID_CANOPEN_OD_ERROR_REGISTER            0x1001U /**< 错误寄存器。 */
#define MID_CANOPEN_OD_PREDEFINED_ERROR_FIELD    0x1003U /**< 预定义错误字段。 */
#define MID_CANOPEN_OD_SYNC_COB_ID               0x1005U /**< 同步帧 COB-ID。 */
#define MID_CANOPEN_OD_SYNC_CYCLE_PERIOD         0x1006U /**< 同步循环周期，单位毫秒。 */
#define MID_CANOPEN_OD_DEVICE_NAME               0x1008U /**< 设备名称。 */
#define MID_CANOPEN_OD_HARDWARE_VERSION          0x1009U /**< 硬件版本。 */
#define MID_CANOPEN_OD_SOFTWARE_VERSION          0x100AU /**< 软件版本。 */
#define MID_CANOPEN_OD_STORE_PARAMETERS          0x1010U /**< 保存参数。 */
#define MID_CANOPEN_OD_RESTORE_DEFAULT           0x1011U /**< 恢复出厂参数。 */
#define MID_CANOPEN_OD_EMCY_COB_ID               0x1014U /**< 紧急报文 COB-ID。 */
#define MID_CANOPEN_OD_HEARTBEAT_TIME            0x1017U /**< 心跳产生周期，单位毫秒。 */
#define MID_CANOPEN_OD_IDENTITY                  0x1018U /**< 厂商与产品标识。 */
#define MID_CANOPEN_OD_SDO_SERVER                0x1200U /**< SDO 服务器参数。 */
#define MID_CANOPEN_OD_RPDO_COMM_PARAM           0x1400U /**< RPDO1 至 4 通信参数。 */
#define MID_CANOPEN_OD_RPDO_MAPPING              0x1600U /**< RPDO1 至 4 映射参数。 */
#define MID_CANOPEN_OD_TPDO_COMM_PARAM           0x1800U /**< TPDO1 至 4 通信参数。 */
#define MID_CANOPEN_OD_TPDO_MAPPING              0x1A00U /**< TPDO1 至 4 映射参数。 */
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_OD_Communication_SubIndex CANopen Communication Profile Sub-Index
  * @{
  */
#define MID_CANOPEN_OD_SDO_SUB_SERVER_RX_COB_ID  0x01U /**< 服务器接收 SDO COB-ID。 */
#define MID_CANOPEN_OD_SDO_SUB_CLIENT_TX_COB_ID  0x02U /**< 客户端发送 SDO COB-ID。 */
#define MID_CANOPEN_OD_STORE_SUB_ALL             0x01U /**< 保存全部参数。 */
#define MID_CANOPEN_OD_STORE_SUB_COMMUNICATION   0x02U /**< 保存通信参数。 */
#define MID_CANOPEN_OD_STORE_SUB_MOTION          0x03U /**< 保存运动参数。 */
#define MID_CANOPEN_OD_STORE_SUB_VENDOR          0x04U /**< 保存厂商参数。 */
#define MID_CANOPEN_OD_RESTORE_SUB_ALL           0x01U /**< 恢复全部出厂参数。 */
#define MID_CANOPEN_OD_RESTORE_SUB_COMMUNICATION 0x02U /**< 恢复通信出厂参数。 */
#define MID_CANOPEN_OD_RESTORE_SUB_MOTION        0x03U /**< 恢复运动出厂参数。 */
#define MID_CANOPEN_OD_RESTORE_SUB_USER          0x04U /**< 恢复用户出厂参数。 */
#define MID_CANOPEN_OD_PDO_SUB_MAP_COUNT         0x00U /**< 映射对象个数。 */
#define MID_CANOPEN_OD_PDO_SUB_MAP_ITEM_1        0x01U /**< 第 1 个映射对象。 */
#define MID_CANOPEN_OD_PDO_SUB_MAP_ITEM_2        0x02U /**< 第 2 个映射对象。 */
#define MID_CANOPEN_OD_PDO_SUB_MAP_ITEM_3        0x03U /**< 第 3 个映射对象。 */
#define MID_CANOPEN_OD_PDO_SUB_MAP_ITEM_4        0x04U /**< 第 4 个映射对象。 */
#define MID_CANOPEN_OD_PDO_SUB_MAP_ITEM_5        0x05U /**< 第 5 个映射对象。 */
#define MID_CANOPEN_OD_PDO_SUB_MAP_ITEM_6        0x06U /**< 第 6 个映射对象。 */
#define MID_CANOPEN_OD_PDO_SUB_MAP_ITEM_7        0x07U /**< 第 7 个映射对象。 */
#define MID_CANOPEN_OD_PDO_SUB_MAP_ITEM_8        0x08U /**< 第 8 个映射对象。 */
#define MID_CANOPEN_OD_PDO_SUB_COB_ID            0x01U /**< PDO 使用的 COB-ID。 */
#define MID_CANOPEN_OD_PDO_SUB_TRANSMISSION_TYPE 0x02U /**< PDO 传输类型。 */
#define MID_CANOPEN_OD_PDO_SUB_INHIBIT_TIME      0x03U /**< PDO 禁止时间。 */
#define MID_CANOPEN_OD_PDO_SUB_COMPATIBILITY     0x04U /**< 兼容性入口。 */
#define MID_CANOPEN_OD_PDO_SUB_EVENT_TIMER       0x05U /**< PDO 事件定时器周期，单位毫秒。 */
#define MID_CANOPEN_OD_PDO_SUB_SYNC_START_VALUE  0x06U /**< 同步起始值。 */
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_OD_Manufacturer_Area CANopen Manufacturer Specific Area
  * @{
  */
#define MID_CANOPEN_OD_PEAK_CURRENT              0x2000U /**< 驱动器峰值电流，单位毫安。 */
#define MID_CANOPEN_OD_MICRO_STEP                0x2001U /**< 电机运行一圈所需脉冲数。 */
#define MID_CANOPEN_OD_SMOOTH_TIME               0x2002U /**< 平滑时间，单位毫秒。 */
#define MID_CANOPEN_OD_MINIMUM_CURRENT           0x2003U /**< 最小电流参数。 */
#define MID_CANOPEN_OD_OUTPUT_FUNCTION           0x2005U /**< 输出端口功能设置。 */
#define MID_CANOPEN_OD_ENCODER_LINES             0x2007U /**< 编码器线数。 */
#define MID_CANOPEN_OD_OUTPUT_ACTIVE_LEVEL       0x2008U /**< 输出口有效电平。 */
#define MID_CANOPEN_OD_POSITION_WARNING_THRESHOLD 0x2009U /**< 位置告警阀值，单位步。 */
#define MID_CANOPEN_OD_POSITION_ERROR_THRESHOLD  0x2010U /**< 位置超差阀值，单位步。 */
#define MID_CANOPEN_OD_MOTOR_DIRECTION           0x2051U /**< 电机运行方向。 */
#define MID_CANOPEN_OD_CLEAR_CURRENT_ALARM       0x2057U /**< 清除当前报警。 */
#define MID_CANOPEN_OD_CLEAR_FAULT_RECORD        0x2093U /**< 清除故障记录。 */
#define MID_CANOPEN_OD_SLAVE_ADDRESS             0x2150U /**< 软件从站地址。 */
#define MID_CANOPEN_OD_ADDRESS_SOURCE            0x2151U /**< 从站地址来源。 */
#define MID_CANOPEN_OD_INPUT_FUNCTION            0x2152U /**< 输入信号端口定义。 */
#define MID_CANOPEN_OD_INPUT_FILTER              0x2153U /**< 输入端口滤波时间，单位毫秒。 */
#define MID_CANOPEN_OD_INPUT_POLARITY            0x2154U /**< 输入端口极性设置。 */
#define MID_CANOPEN_OD_INPUT_STATUS              0x2155U /**< 输入口状态。 */
#define MID_CANOPEN_OD_TORQUE_CONSTANT           0x2157U /**< 力矩常数，单位 m.Nm/A。 */
#define MID_CANOPEN_OD_TORQUE_PARAMETER          0x2158U /**< 力矩模式参数。 */
#define MID_CANOPEN_OD_CAN_BAUDRATE              0x2180U /**< CAN 通讯波特率设置。 */
#define MID_CANOPEN_OD_HISTORY_FAULT_1           0x3000U /**< 历史故障 1。 */
#define MID_CANOPEN_OD_HISTORY_FAULT_2           0x3001U /**< 历史故障 2。 */
#define MID_CANOPEN_OD_HISTORY_FAULT_3           0x3002U /**< 历史故障 3。 */
#define MID_CANOPEN_OD_HISTORY_FAULT_4           0x3003U /**< 历史故障 4。 */
#define MID_CANOPEN_OD_PROGRAM_SEGMENT           0x3100U /**< 执行指定程序段。 */
#define MID_CANOPEN_OD_MOTOR_ENABLE_SELECT       0x3200U /**< 上电电机使能选择。 */
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_OD_Manufacturer_SubIndex CANopen Manufacturer Specific Sub-Index
  * @{
  */
#define MID_CANOPEN_OD_OUTPUT_SUB_OUT1           0x01U /**< 输出 1 功能设置。 */
#define MID_CANOPEN_OD_OUTPUT_SUB_OUT2           0x02U /**< 输出 2 功能设置。 */
#define MID_CANOPEN_OD_OUTPUT_SUB_OUT3           0x03U /**< 输出 3 功能设置。 */
#define MID_CANOPEN_OD_OUTPUT_SUB_OUT4           0x04U /**< 输出 4 功能设置。 */
#define MID_CANOPEN_OD_INPUT_SUB_IN1             0x01U /**< IN1 配置。 */
#define MID_CANOPEN_OD_INPUT_SUB_IN2             0x02U /**< IN2 配置。 */
#define MID_CANOPEN_OD_INPUT_SUB_IN3             0x03U /**< IN3 配置。 */
#define MID_CANOPEN_OD_INPUT_SUB_IN4             0x04U /**< IN4 配置。 */
#define MID_CANOPEN_OD_INPUT_SUB_IN5             0x05U /**< IN5 配置。 */
#define MID_CANOPEN_OD_INPUT_SUB_POLARITY        0x01U /**< 输入口有效电平。 */
#define MID_CANOPEN_OD_INPUT_SUB_INVERT          0x02U /**< 输入口状态取反。 */
#define MID_CANOPEN_OD_TORQUE_SUB_MODE           0x01U /**< 力矩工作方式。 */
#define MID_CANOPEN_OD_TORQUE_SUB_OFFSET_PULSE   0x02U /**< 偏移脉冲。 */
#define MID_CANOPEN_OD_TORQUE_SUB_MAX_SPEED      0x03U /**< 力矩模式速度限制。 */
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_OD_Device_Profile CANopen Device Profile Area (CiA402)
  * @{
  */
#define MID_CANOPEN_OD_ERROR_CODE                0x603FU /**< 错误代码。 */
#define MID_CANOPEN_OD_CONTROL_WORD              0x6040U /**< 控制字。 */
#define MID_CANOPEN_OD_STATUS_WORD               0x6041U /**< 状态字。 */
#define MID_CANOPEN_OD_MODE_OF_OPERATION         0x6060U /**< 写运行模式。 */
#define MID_CANOPEN_OD_MODE_OF_OPERATION_DISPLAY 0x6061U /**< 读运行模式。 */
#define MID_CANOPEN_OD_POSITION_ACTUAL_VALUE     0x6064U /**< 实际位置，单位脉冲。 */
#define MID_CANOPEN_OD_VELOCITY_ACTUAL_VALUE     0x606CU /**< 实际速度，单位脉冲每秒。 */
#define MID_CANOPEN_OD_TARGET_TORQUE             0x6071U /**< 目标力矩。 */
#define MID_CANOPEN_OD_MAX_TORQUE                0x6072U /**< 最大力矩。 */
#define MID_CANOPEN_OD_MOTOR_RATED_TORQUE        0x6076U /**< 马达额定力矩。 */
#define MID_CANOPEN_OD_TORQUE_ACTUAL_VALUE       0x6077U /**< 实际力矩。 */
#define MID_CANOPEN_OD_CURRENT_ACTUAL_VALUE      0x6078U /**< 实时电流，单位毫安。 */
#define MID_CANOPEN_OD_TARGET_POSITION           0x607AU /**< 目标位置，单位脉冲。 */
#define MID_CANOPEN_OD_PROFILE_VELOCITY          0x6081U /**< 梯形速度，单位脉冲每秒。 */
#define MID_CANOPEN_OD_PROFILE_ACCELERATION      0x6083U /**< 梯形加速度，单位脉冲每平方秒。 */
#define MID_CANOPEN_OD_PROFILE_DECELERATION      0x6084U /**< 梯形减速度，单位脉冲每平方秒。 */
#define MID_CANOPEN_OD_HOMING_METHOD             0x6098U /**< 回原点模式。 */
#define MID_CANOPEN_OD_HOMING_SPEEDS             0x6099U /**< 回原点速度。 */
#define MID_CANOPEN_OD_HOMING_ACCELERATION       0x609AU /**< 回原点加速度，单位脉冲每平方秒。 */
#define MID_CANOPEN_OD_INTERPOLATION_TARGET_POSITION 0x60C1U /**< 插补模式周期目标位置。 */
#define MID_CANOPEN_OD_INTERPOLATION_TIME_PERIOD 0x60C2U /**< 插补周期，单位毫秒。 */
#define MID_CANOPEN_OD_DIGITAL_INPUTS            0x60FDU /**< 32 位输入检测及电机状态。 */
#define MID_CANOPEN_OD_DIGITAL_OUTPUTS           0x60FEU /**< 3 位输出控制。 */
#define MID_CANOPEN_OD_TARGET_VELOCITY           0x60FFU /**< 目标速度，单位脉冲每秒。 */
#define MID_CANOPEN_OD_SUPPORTED_DRIVE_MODES     0x6502U /**< 支持的驱动模式种类。 */
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_OD_Device_Profile_SubIndex CANopen Device Profile Sub-Index
  * @{
  */
#define MID_CANOPEN_OD_SUB_INDEX_0               0x00U /**< 子索引 0。 */
#define MID_CANOPEN_OD_HOMING_SPEED_SUB_FAST     0x01U /**< 快速回原点速度。 */
#define MID_CANOPEN_OD_HOMING_SPEED_SUB_SLOW     0x02U /**< 慢速回原点速度。 */
#define MID_CANOPEN_OD_INTERPOLATION_TIME_SUB    0x01U /**< 插补周期子索引。 */
#define MID_CANOPEN_OD_DIGITAL_OUTPUT_SUB_SET    0x00U /**< 输出控制。 */
#define MID_CANOPEN_OD_DIGITAL_OUTPUT_SUB_ENABLE 0x02U /**< 输出使能。 */
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_OD_Store_Command CANopen Store and Restore Command
  * @{
  */
#define MID_CANOPEN_OD_STORE_SIGNATURE           0x65766173U /**< 保存参数签名，对应 ASCII 字符 save。 */
#define MID_CANOPEN_OD_RESTORE_SIGNATURE         0x64616F6CU /**< 恢复出厂签名，对应 ASCII 字符 load。 */
/**
  * @}
  */

/**
  * @defgroup MID_CANOPEN_OD_PDO_Mapping_Entry CANopen PDO Mapping Entry
  * @{
  */
#define MID_CANOPEN_OD_PDO_SIZE_8BIT             0x08U /**< 映射对象长度，8 位。 */
#define MID_CANOPEN_OD_PDO_SIZE_16BIT            0x10U /**< 映射对象长度，16 位。 */
#define MID_CANOPEN_OD_PDO_SIZE_32BIT            0x20U /**< 映射对象长度，32 位。 */

/**
  * @brief  构造 PDO 映射参数，位域为索引 16 位、子索引 8 位、对象长度 8 位。
  * @param  index      对象字典索引，取值参见 @ref MID_CANOPEN_OD_Communication_Profile、
  *                    @ref MID_CANOPEN_OD_Manufacturer_Area 与 @ref MID_CANOPEN_OD_Device_Profile。
  * @param  subIndex   对象字典子索引，取值参见 @ref MID_CANOPEN_OD_Communication_SubIndex、
  *                    @ref MID_CANOPEN_OD_Manufacturer_SubIndex 与 @ref MID_CANOPEN_OD_Device_Profile_SubIndex。
  * @param  bitLength  对象长度，取 MID_CANOPEN_OD_PDO_SIZE_xxx，取值参见 @ref MID_CANOPEN_OD_PDO_Mapping_Entry。
  */
#define MID_CANOPEN_OD_PDO_MAP_ENTRY(index, subIndex, bitLength)                     \
  ((((uint32_t)(index)) << 16U) | (((uint32_t)(subIndex)) << 8U) | ((uint32_t)(bitLength)))
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __MID_CANOPEN_OD_H */
