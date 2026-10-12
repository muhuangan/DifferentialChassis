/**
  ******************************************************************************
  * @file    mid_lisan_ec.h
  * @brief   立三 EC 系列 CANopen 步进驱动器控制接口。
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
  * 本模块基于 mid_canopen 协议栈，按立三 EC 系列驱动器 CANopen 通讯手册 V4.02 封装
  * CiA402 驱动状态机、对象字典参数与工作模式控制。
  *
  ******************************************************************************
  */

#ifndef __MID_LISAN_EC_H
#define __MID_LISAN_EC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "mid_canopen.h"

/**
  * @defgroup MID_LISAN_EC_Operation_Mode Lisan EC Operation Mode
  * @{
  */
typedef enum
{
  MID_LISAN_EC_MODE_PROFILE_POSITION = 0x01U, /**< 轮廓位置模式，对应 0x6060 写入 1。 */
  MID_LISAN_EC_MODE_PROFILE_VELOCITY = 0x03U, /**< 轮廓速度模式，对应 0x6060 写入 3。 */
  MID_LISAN_EC_MODE_PROFILE_TORQUE   = 0x04U, /**< 力矩模式，对应 0x6060 写入 4，仅闭环机型支持。 */
  MID_LISAN_EC_MODE_HOMING           = 0x06U  /**< 回原点模式，对应 0x6060 写入 6。 */
} MID_LisanEcModeTypeDef;
/**
  * @}
  */

/**
  * @defgroup MID_LISAN_EC_Torque_Mode Lisan EC Torque Work Mode
  * @{
  */
typedef enum
{
  MID_LISAN_EC_TORQUE_MODE_CONSTANT      = 0x01U, /**< 恒力矩运行。 */
  MID_LISAN_EC_TORQUE_MODE_COLLISION_HOME = 0x02U, /**< 碰撞回原点。 */
  MID_LISAN_EC_TORQUE_MODE_GRIP_OBJECT   = 0x03U  /**< 抓取物体。 */
} MID_LisanEcTorqueModeTypeDef;
/**
  * @}
  */

/**
  * @defgroup MID_LISAN_EC_Control_Word Lisan EC Control Word 0x6040
  * @{
  */
#define MID_LISAN_EC_CONTROL_WORD_MOTOR_FREE        0x0006U      /**< bit1 置位，脱机取消锁轴。 */
#define MID_LISAN_EC_CONTROL_WORD_MOTOR_LOCK        0x0007U      /**< bit0 至 bit2 置位，锁机启动状态。 */
#define MID_LISAN_EC_CONTROL_WORD_ENABLE_OPERATION  0x000FU      /**< bit0 至 bit3 置位，接通主电允许运行。 */
#define MID_LISAN_EC_CONTROL_WORD_QUICK_STOP        0x0002U      /**< bit2 为 0，启动快速停机。 */
#define MID_LISAN_EC_CONTROL_WORD_FAULT_RESET       0x0080U      /**< bit7 上升沿复位可复位故障。 */
#define MID_LISAN_EC_CONTROL_WORD_HALT              0x0100U      /**< bit8 置位，电机轴减速停止。 */
#define MID_LISAN_EC_CONTROL_WORD_NEW_SETPOINT      (1U << 4U)   /**< bit4 由 0 跳变为 1 触发新的位置指令。 */
#define MID_LISAN_EC_CONTROL_WORD_SETPOINT_NOW      (1U << 5U)   /**< bit5 置位表示中断当前指令立即执行新位置。 */
#define MID_LISAN_EC_CONTROL_WORD_RELATIVE_POSITION (1U << 6U)   /**< bit6 置位表示目标位置为相对位置。 */
/**
  * @}
  */

/**
  * @defgroup MID_LISAN_EC_Status_Word Lisan EC Status Word 0x6041
  * @{
  */
#define MID_LISAN_EC_STATUS_WORD_FAULT_FREE          (1U << 0U)  /**< bit0 驱动无故障。 */
#define MID_LISAN_EC_STATUS_WORD_READY_TO_SWITCH_ON  (1U << 1U)  /**< bit1 等待打开驱动使能。 */
#define MID_LISAN_EC_STATUS_WORD_OPERATION_ENABLED   (1U << 2U)  /**< bit2 驱动运行。 */
#define MID_LISAN_EC_STATUS_WORD_FAULT               (1U << 3U)  /**< bit3 故障。 */
#define MID_LISAN_EC_STATUS_WORD_VOLTAGE_ENABLED     (1U << 4U)  /**< bit4 接通主回路电。 */
#define MID_LISAN_EC_STATUS_WORD_QUICK_STOP          (1U << 5U)  /**< bit5 为 0 表示快速停机激活。 */
#define MID_LISAN_EC_STATUS_WORD_SWITCH_ON_DISABLED  (1U << 6U)  /**< bit6 驱动准备好。 */
#define MID_LISAN_EC_STATUS_WORD_WARNING             (1U << 7U)  /**< bit7 警告。 */
#define MID_LISAN_EC_STATUS_WORD_REMOTE              (1U << 9U)  /**< bit9 CANopen 远程控制模式。 */
#define MID_LISAN_EC_STATUS_WORD_TARGET_REACHED      (1U << 10U) /**< bit10 目标位置到达。 */
#define MID_LISAN_EC_STATUS_WORD_INTERNAL_LIMIT      (1U << 11U) /**< bit11 软件内部位置超限。 */
#define MID_LISAN_EC_STATUS_WORD_HOMING_ATTAINED     (1U << 15U) /**< bit15 原点回零完成。 */
/**
  * @}
  */

/**
  * @defgroup MID_LISAN_EC_State_Code Lisan EC CiA402 State Code
  * @{
  */
#define MID_LISAN_EC_STATE_MASK_LOW         0x004FU /**< bit0 至 bit3 与 bit6 组成的低位状态掩码。 */
#define MID_LISAN_EC_STATE_MASK_FULL        0x006FU /**< bit0 至 bit3、bit5 与 bit6 组成的掩码。 */
#define MID_LISAN_EC_STATE_CODE_NOT_READY_TO_SWITCH_ON 0x0000U /**< 状态字低位取值，尚未准备就绪。 */
#define MID_LISAN_EC_STATE_CODE_SWITCH_ON_DISABLED     0x0040U /**< 状态字低位取值，伺服无故障。 */
#define MID_LISAN_EC_STATE_CODE_READY_TO_SWITCH_ON     0x0021U /**< 状态字低位取值，伺服准备好。 */
#define MID_LISAN_EC_STATE_CODE_SWITCHED_ON            0x0023U /**< 状态字低位取值，等待打开伺服使能。 */
#define MID_LISAN_EC_STATE_CODE_OPERATION_ENABLED      0x0027U /**< 状态字低位取值，伺服运行。 */
#define MID_LISAN_EC_STATE_CODE_QUICK_STOP_ACTIVE      0x0007U /**< 状态字低位取值，快速停机。 */
#define MID_LISAN_EC_STATE_CODE_FAULT_REACTION_ACTIVE  0x000FU /**< 状态字低位取值，故障停机。 */
#define MID_LISAN_EC_STATE_CODE_FAULT                  0x0008U /**< 状态字低位取值，故障。 */
/**
  * @}
  */

/**
  * @defgroup MID_LISAN_EC_Direction Lisan EC Motor Direction
  * @{
  */
#define MID_LISAN_EC_DIRECTION_CW_POSITIVE   0x0000U /**< 正方向为顺时针旋转。 */
#define MID_LISAN_EC_DIRECTION_CCW_POSITIVE  0x0001U /**< 正方向为逆时针旋转。 */
/**
  * @}
  */

/**
  * @defgroup MID_LISAN_EC_Address_Source Lisan EC Node Address Source
  * @{
  */
#define MID_LISAN_EC_ADDRESS_SOURCE_DIAL_PLUS_OBJECT 0x0000U /**< 实际地址来源于拨码开关与 0x2150 之和。 */
#define MID_LISAN_EC_ADDRESS_SOURCE_OBJECT_ONLY      0x0001U /**< 实际地址来源于 0x2150。 */
/**
  * @}
  */

/**
  * @defgroup MID_LISAN_EC_Homing_Method Lisan EC Homing Method
  * @{
  */
#define MID_LISAN_EC_HOMING_METHOD_MIN       17U /**< 支持的回原点方式下限。 */
#define MID_LISAN_EC_HOMING_METHOD_MAX       30U /**< 支持的回原点方式上限。 */
#define MID_LISAN_EC_HOMING_METHOD_SET_ZERO_35 35U /**< 不运行，直接把当前位置置零。 */
#define MID_LISAN_EC_HOMING_METHOD_SET_ZERO_37 37U /**< 不运行，直接把当前位置置零。 */
/**
  * @}
  */

/**
  * @defgroup MID_LISAN_EC_Error_Code Lisan EC Error Code 0x603F
  * @{
  */
#define MID_LISAN_EC_ERROR_CODE_OVER_CURRENT     0x2211U /**< 驱动器过流。 */
#define MID_LISAN_EC_ERROR_CODE_MOTOR_OPEN       0x7122U /**< 相位开路。 */
#define MID_LISAN_EC_ERROR_CODE_VOLTAGE_TOO_HIGH 0x3211U /**< 供电电压过高。 */
#define MID_LISAN_EC_ERROR_CODE_VOLTAGE_TOO_LOW  0x3122U /**< 供电电压过低。 */
#define MID_LISAN_EC_ERROR_CODE_PHASE_A_DRIFT    0x3150U /**< A 相电压漂移超过容忍值。 */
#define MID_LISAN_EC_ERROR_CODE_PHASE_B_DRIFT    0x3151U /**< B 相电压漂移超过容忍值。 */
#define MID_LISAN_EC_ERROR_CODE_POSITION_ERROR   0x8403U /**< 位置误差，多发生在闭环机型。 */
#define MID_LISAN_EC_ERROR_CODE_ENCODER_ERROR    0x7300U /**< 编码器错误，仅闭环机型。 */
/**
  * @}
  */

/**
  * @defgroup MID_LISAN_EC_Drive_State Lisan EC Drive State
  * @{
  */
typedef enum
{
  MID_LISAN_EC_STATE_UNKNOWN                = 0x00U, /**< 状态未识别。 */
  MID_LISAN_EC_STATE_NOT_READY_TO_SWITCH_ON = 0x01U, /**< 驱动器初始化过程中。 */
  MID_LISAN_EC_STATE_SWITCH_ON_DISABLED     = 0x02U, /**< 伺服无故障，参数可设置。 */
  MID_LISAN_EC_STATE_READY_TO_SWITCH_ON     = 0x03U, /**< 伺服准备好。 */
  MID_LISAN_EC_STATE_SWITCHED_ON            = 0x04U, /**< 等待打开伺服使能。 */
  MID_LISAN_EC_STATE_OPERATION_ENABLED      = 0x05U, /**< 伺服运行。 */
  MID_LISAN_EC_STATE_QUICK_STOP_ACTIVE      = 0x06U, /**< 快速停机执行中。 */
  MID_LISAN_EC_STATE_FAULT_REACTION_ACTIVE  = 0x07U, /**< 故障停机执行中。 */
  MID_LISAN_EC_STATE_FAULT                  = 0x08U  /**< 故障，可通过控制字复位。 */
} MID_LisanEcStateTypeDef;
/**
  * @}
  */

/**
  * @brief  状态字 0x6041 解析结果。
  */
typedef struct
{
  uint16_t raw;                    /**< 状态字原始数值。 */
  bool     isFault;                /**< bit3 故障。 */
  bool     isFaultFree;            /**< bit0 驱动无故障。 */
  bool     isReadyToSwitchOn;      /**< bit1 等待打开驱动使能。 */
  bool     isOperationEnabled;     /**< bit2 驱动运行。 */
  bool     isVoltageEnabled;       /**< bit4 接通主回路电。 */
  bool     isQuickStopActive;      /**< bit5 为 0 表示快速停机激活。 */
  bool     isSwitchOnDisabled;     /**< bit6 驱动准备好。 */
  bool     isWarning;              /**< bit7 警告。 */
  bool     isRemote;               /**< bit9 CANopen 远程控制模式。 */
  bool     isTargetReached;        /**< bit10 目标位置到达。 */
  bool     isInternalLimitActive;  /**< bit11 软件内部位置超限。 */
  bool     isHomingAttained;       /**< bit15 原点回零完成。 */
} MID_LisanEcStatusWordTypeDef;

/**
  * @brief  使能驱动器。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 使能成功。
  * @retval 其他          SDO 接口错误码。
  * @note   依次写入控制字 0x06、0x07、0x0F，完成 CiA402 状态机切换。
  */
MID_CANopenStatusTypeDef MID_LisanEc_EnableDrive(uint8_t nodeId);

/**
  * @brief  写入控制字 0x6040。
  * @param  nodeId       节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                      取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  controlWord  控制字数值，写入 0x6040，取值参见 @ref MID_LISAN_EC_Control_Word。
  * @retval MID_CANOPEN_OK 写入成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_WriteControlWord(uint8_t nodeId, uint16_t controlWord);

/**
  * @brief  读取控制字 0x6040。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pControlWord  指向接收 0x6040 控制字原始数值的变量的指针，取值参见 @ref MID_LISAN_EC_Control_Word，
  *                       不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadControlWord(uint8_t nodeId, uint16_t *pControlWord);

/**
  * @brief  电机脱机，取消锁轴状态。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMotorFree(uint8_t nodeId);

/**
  * @brief  电机锁机，锁定在当前位置。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMotorLock(uint8_t nodeId);

/**
  * @brief  启动快速停机。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_QuickStop(uint8_t nodeId);

/**
  * @brief  减速停止。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  * @note   清除减速停止信号后电机运行状态不恢复，需要重新切换状态机。
  */
MID_CANopenStatusTypeDef MID_LisanEc_Halt(uint8_t nodeId);

/**
  * @brief  复位可复位故障，清除控制字的 bit4 至 bit6。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 复位指令已发出。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_FaultReset(uint8_t nodeId);

/**
  * @brief  设置工作模式 0x6060。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  mode    工作模式，取 MID_LisanEcModeTypeDef 枚举值，取值参见 @ref MID_LISAN_EC_Operation_Mode。
  * @retval MID_CANOPEN_OK            设置成功。
  * @retval MID_CANOPEN_INVALID_PARAM 模式非法。
  * @retval 其他                      SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetOperationMode(uint8_t nodeId, MID_LisanEcModeTypeDef mode);

/**
  * @brief  读取当前工作模式 0x6061。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pMode   指向接收 0x6061 当前工作模式的变量的指针，取值参见 @ref MID_LISAN_EC_Operation_Mode，不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadOperationMode(uint8_t nodeId, MID_LisanEcModeTypeDef *pMode);

/**
  * @brief  设置电机细分 0x2001。
  * @param  nodeId         节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                        取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pulsePerRound  电机运行一圈所需脉冲数，写入 0x2001，由细分倍数与编码器线数共同决定。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMicroStep(uint8_t nodeId, uint32_t pulsePerRound);

/**
  * @brief  设置驱动器峰值电流 0x2000。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  currentMa  峰值电流，单位毫安，写入 0x2000，不得超过电机额定电流。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  * @note   电流设置过大可能损坏电机，需按电机额定电流设置。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetPeakCurrent(uint8_t nodeId, uint16_t currentMa);

/**
  * @brief  设置平滑时间 0x2002。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  smoothTimeMs  平滑时间，单位毫秒，写入 0x2002，驱动器接受的范围为 1 至 1500。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetSmoothTime(uint8_t nodeId, uint16_t smoothTimeMs);

/**
  * @brief  设置最小电流参数 0x2003。
  * @param  nodeId          节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                         取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  minimumCurrent  最小电流参数原始值，写入 0x2003；开环机型低 8 位为半流状态电流百分比，
  *                         闭环机型高低字节含义不同。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他              SDO 接口错误码。
  * @note   开环机型低 8 位表示半流状态电流百分比，闭环机型高低字节含义不同，需按机型换算。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMinimumCurrent(uint8_t nodeId, uint16_t minimumCurrent);

/**
  * @brief  设置电机运行方向 0x2051。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  direction  电机运行方向，取 MID_LISAN_EC_DIRECTION_xxx，取值参见 @ref MID_LISAN_EC_Direction。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  * @note   不可在运行状态下修改。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMotorDirection(uint8_t nodeId, uint16_t direction);

/**
  * @brief  设置梯形加速度 0x6083。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  acceleration  梯形加速度，单位脉冲每平方秒，写入 0x6083。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetProfileAcceleration(uint8_t nodeId, uint32_t acceleration);

/**
  * @brief  设置梯形减速度 0x6084。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  deceleration  梯形减速度，单位脉冲每平方秒，写入 0x6084。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetProfileDeceleration(uint8_t nodeId, uint32_t deceleration);

/**
  * @brief  设置位置模式梯形速度 0x6081。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  velocity  位置模式下允许的最大速度，单位脉冲每秒，写入 0x6081，只有大小没有方向。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  * @note   位置模式下速度只有大小，方向由目标位置决定。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetProfileVelocity(uint8_t nodeId, uint32_t velocity);

/**
  * @brief  设置目标位置 0x607A。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  position  目标位置，单位脉冲，写入 0x607A。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetTargetPosition(uint8_t nodeId, int32_t position);

/**
  * @brief  设置目标速度 0x60FF。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  velocity  目标速度，单位脉冲每秒，写入 0x60FF，符号表示方向。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetTargetVelocity(uint8_t nodeId, int32_t velocity);

/**
  * @brief  设置目标力矩 0x6071。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  torque  力矩等级，写入 0x6071，数值表示大小，符号表示方向，取值参见 @ref MID_LISAN_EC_Torque_Range。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  * @note   运行中只能修改力矩大小，方向修改无效。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetTargetTorque(uint8_t nodeId, int16_t torque);

/**
  * @brief  设置最大力矩 0x6072。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  maxTorque  最大力矩等级，写入 0x6072，数值越大可达到的最大电流越大，
  *                    取值参见 @ref MID_LISAN_EC_Torque_Range。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetMaxTorque(uint8_t nodeId, uint16_t maxTorque);

/**
  * @brief  配置力矩模式参数 0x2158。
  * @param  nodeId       节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                      取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  torqueMode   力矩工作方式，取 MID_LisanEcTorqueModeTypeDef 枚举值，写入 0x2158 子索引 1，
  *                      取值参见 @ref MID_LISAN_EC_Torque_Mode。
  * @param  offsetPulse  偏移脉冲，写入 0x2158 子索引 2，碰撞回原点与抓取物体时有效，
  *                      取值参见 @ref MID_LISAN_EC_Torque_Range。
  * @param  maxSpeed     力矩模式速度限制，单位转每分钟，写入 0x2158 子索引 3。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ConfigureTorqueMode(uint8_t nodeId, MID_LisanEcTorqueModeTypeDef torqueMode,
                                                         uint16_t offsetPulse, uint16_t maxSpeed);

/**
  * @brief  配置回原点参数。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  method        回原点方式，写入 0x6098，取值参见 @ref MID_LISAN_EC_Homing_Method。
  * @param  fastSpeed     快速回原点速度，单位脉冲每秒，写入 0x6099 子索引 1，最高位表示方向。
  * @param  slowSpeed     慢速回原点速度，单位脉冲每秒，写入 0x6099 子索引 2，最高位表示方向。
  * @param  acceleration  回原点加速度，单位脉冲每平方秒，写入 0x609A。
  * @retval MID_CANOPEN_OK 设置成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ConfigureHoming(uint8_t nodeId, uint8_t method, uint32_t fastSpeed,
                                                     uint32_t slowSpeed, uint32_t acceleration);

/**
  * @brief  以绝对位置方式启动轮廓位置模式运动。
  * @param  nodeId       节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                      取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  position     目标位置，单位脉冲，写入 0x607A。
  * @param  isImmediate  true 表示中断当前指令立即执行，false 表示等待当前指令执行完成后再生效。
  * @retval MID_CANOPEN_OK 指令已下发。
  * @retval 其他          SDO 接口错误码。
  * @note   调用前需完成模式设置与使能，并已设置好加减速与梯形速度。
  */
MID_CANopenStatusTypeDef MID_LisanEc_MoveAbsolute(uint8_t nodeId, int32_t position, bool isImmediate);

/**
  * @brief  以相对位置方式启动轮廓位置模式运动。
  * @param  nodeId         节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                        取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  deltaPosition  相对位移，单位脉冲，写入 0x607A，符号表示方向。
  * @param  isImmediate    true 表示中断当前指令立即执行，false 表示等待当前指令执行完成后再生效。
  * @retval MID_CANOPEN_OK 指令已下发。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_MoveRelative(uint8_t nodeId, int32_t deltaPosition, bool isImmediate);

/**
  * @brief  清除新的位置指令。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 操作成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ClearSetpoint(uint8_t nodeId);

/**
  * @brief  启动回原点运动。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 指令已下发。
  * @retval 其他          SDO 接口错误码。
  * @note   内部按 0x07、0x0F、0x1F 顺序下发控制字。
  */
MID_CANopenStatusTypeDef MID_LisanEc_StartHoming(uint8_t nodeId);

/**
  * @brief  读取并解析状态字 0x6041。
  * @param  nodeId   节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                  取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pStatus  指向接收 0x6041 状态字解析结果的指针，取值参见 @ref MID_LISAN_EC_Status_Word，不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadStatusWord(uint8_t nodeId, MID_LisanEcStatusWordTypeDef *pStatus);

/**
  * @brief  读取实际位置 0x6064。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pPosition  指向接收 0x6064 实际位置的变量的指针，单位脉冲，不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadPosition(uint8_t nodeId, int32_t *pPosition);

/**
  * @brief  读取实际速度 0x606C。
  * @param  nodeId     节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                    取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pVelocity  指向接收 0x606C 实际速度的变量的指针，单位脉冲每秒，不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadVelocity(uint8_t nodeId, int32_t *pVelocity);

/**
  * @brief  读取实时电流 0x6078。
  * @param  nodeId    节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                   取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pCurrent  指向接收 0x6078 实时电流的变量的指针，单位毫安，不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadCurrent(uint8_t nodeId, int16_t *pCurrent);

/**
  * @brief  读取驱动器错误代码 0x603F。
  * @param  nodeId      节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                     取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pErrorCode  指向接收 0x603F 错误代码的变量的指针，取值参见 @ref MID_LISAN_EC_Error_Code，不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadErrorCode(uint8_t nodeId, uint16_t *pErrorCode);

/**
  * @brief  读取输入口与电机状态 0x60FD。
  * @param  nodeId   节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                  取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  pInputs  指向接收 0x60FD 输入口与电机状态位图的变量的指针，bit0 至 bit7 对应 IN1 至 IN8，不可为 NULL。
  * @retval MID_CANOPEN_OK 读取成功。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_ReadDigitalInputs(uint8_t nodeId, uint32_t *pInputs);

/**
  * @brief  设置从站地址 0x2150 与地址来源 0x2151。
  * @param  nodeId        节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  newAddress    新的从站地址，写入 0x2150，
  *                       合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                       取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @param  isFromObject  true 表示地址来源为对象字典 0x2150，false 表示拨码开关与 0x2150 之和，
  *                       取值参见 @ref MID_LISAN_EC_Address_Source。
  * @retval MID_CANOPEN_OK            设置成功。
  * @retval MID_CANOPEN_INVALID_PARAM 地址非法。
  * @retval 其他                      SDO 接口错误码。
  * @note   掉电重启后生效，需配合 MID_LisanEc_StoreParameters 保存。
  */
MID_CANopenStatusTypeDef MID_LisanEc_SetNodeAddress(uint8_t nodeId, uint16_t newAddress, bool isFromObject);

/**
  * @brief  保存全部参数 0x1010 子索引 1。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 保存指令已发出。
  * @retval 其他          SDO 接口错误码。
  * @note   驱动器保存完成后重启，通信会短暂中断。
  */
MID_CANopenStatusTypeDef MID_LisanEc_StoreParameters(uint8_t nodeId);

/**
  * @brief  恢复全部出厂参数 0x1011 子索引 1。
  * @param  nodeId  节点地址，合法范围 MID_CANOPEN_NODE_ID_MIN 至 MID_CANOPEN_NODE_ID_MAX（即 1 至 127），
  *                 取值参见 @ref MID_CANOPEN_Protocol_Constant。
  * @retval MID_CANOPEN_OK 恢复指令已发出。
  * @retval 其他          SDO 接口错误码。
  */
MID_CANopenStatusTypeDef MID_LisanEc_RestoreDefault(uint8_t nodeId);

/**
  * @brief  解析状态字得到 CiA402 驱动状态。
  * @param  statusWord  状态字 0x6041 原始数值，取值参见 @ref MID_LISAN_EC_Status_Word。
  * @retval 驱动状态。
  */
MID_LisanEcStateTypeDef MID_LisanEc_ParseState(uint16_t statusWord);

/**
  * @brief  解析状态字到结构体。
  * @param  statusWord  状态字 0x6041 原始数值，取值参见 @ref MID_LISAN_EC_Status_Word。
  * @param  pStatus     指向接收解析结果的指针，成员与 0x6041 各位一一对应，不可为 NULL。
  * @retval None
  */
void MID_LisanEc_ParseStatusWord(uint16_t statusWord, MID_LisanEcStatusWordTypeDef *pStatus);

/**
  * @brief  将错误代码转换为可读文本。
  * @param  errorCode  错误代码 0x603F，取值参见 @ref MID_LISAN_EC_Error_Code。
  * @retval 错误文本，无故障返回 no error，未知代码返回 unknown。
  */
const char *MID_LisanEc_ErrorCodeToString(uint16_t errorCode);

#ifdef __cplusplus
}
#endif

#endif /* __MID_LISAN_EC_H */
