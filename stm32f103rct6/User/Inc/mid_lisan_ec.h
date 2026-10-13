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
  * @note   取值依据手册 4.2「工作模式」入口表（p30）：6060H=1 轮廓位置、6060H=3 轮廓速度、
  *         6060H=4 力矩、6060H=6 回原点，与手册 4.2.1 至 4.2.4 各模式的「主要对象字典」表一致。
  * @note   手册个别示例报文存在笔误：4.2.2 轮廓速度模式示例首帧写成 2F 60 60 00 01（0x6060=1），
  *         4.2.4 力矩模式参数表写成「0x6060=4(03h)」，均与 p30 入口表及其自身参数表矛盾，
  *         本模块以 p30 入口表为准，不随示例报文改动。
  */
typedef enum
{
  MID_LISAN_EC_MODE_PROFILE_POSITION = 0x01U, /**< 轮廓位置模式（PP），对应 0x6060 写入 1。 */
  MID_LISAN_EC_MODE_PROFILE_VELOCITY = 0x03U, /**< 轮廓速度模式（PV），对应 0x6060 写入 3。 */
  MID_LISAN_EC_MODE_PROFILE_TORQUE   = 0x04U, /**< 力矩模式（PT），对应 0x6060 写入 4，仅闭环机型支持。 */
  MID_LISAN_EC_MODE_HOMING           = 0x06U  /**< 回原点模式（HM），对应 0x6060 写入 6。 */
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
  * @note   手册 4.1.2 控制字 6040h（p26）规定：每一个 bit 单独赋值无意义，必须与其他位共同构成一条控制指令；
  *         其中 bit0~bit3 共同组成一条 CiA402 状态机切换命令（手册 4.1.1 控制命令与状态切换表，p25），
  *         bit2 为低有效的快速停机、bit7 为上升沿有效的故障复位、bit8 为减速停止。
  *         因此本组常量一律按「一条完整命令」给出，而不是单独的位掩码。
  * @note   手册自身存在互相矛盾处，本模块按「可执行报文优先、状态机表次之」处理，差异记录如下：
  *         - 减速停止：手册 4.2.2（p37）与 4.2.4（p50）的汇总表均给 010Fh，即 0Fh 叠加 bit8；
  *           而手册 PP/HM 示例报文（p34、p35、p48）写的是 0100h。0100h 的 bit0~bit3 为 0000，
  *           按 4.1.1 属于「伺服运行→伺服无故障」的脱机命令，无法表达「按减速斜坡停止」，
  *           故采用汇总表的 010Fh，保证停止全过程驱动器仍处于伺服运行状态。
  *         - 快速停机：手册 4.1.2（p26）要求满足（**** **** 0*** *01*），手册 4.1.1（p25）状态切换 11
  *           与全部示例报文（p34、p35、p38、p48）均为 0002h；仅手册 4.2.2（p37）汇总表写 00Bh，
  *           且 00Bh 不是 4.1.1 列出的任何一条状态切换命令，判定为汇总表笔误，保留 0002h。
  */
#define MID_LISAN_EC_CONTROL_WORD_MOTOR_FREE        0x0006U      /**< 脱机取消锁轴，对应手册 4.1.1 状态切换 2（伺服无故障→伺服准备好）。 */
#define MID_LISAN_EC_CONTROL_WORD_MOTOR_LOCK        0x0007U      /**< 锁机启动状态，对应手册 4.1.1 状态切换 3（伺服准备好→等待打开伺服使能）。 */
#define MID_LISAN_EC_CONTROL_WORD_ENABLE_OPERATION  0x000FU      /**< 接通主电允许运行，对应手册 4.1.1 状态切换 4（等待打开伺服使能→伺服运行）。 */
#define MID_LISAN_EC_CONTROL_WORD_QUICK_STOP        0x0002U      /**< 快速停机，满足手册 4.1.2 的（**** **** 0*** *01*），即 4.1.1 状态切换 11 的 0x02。 */
#define MID_LISAN_EC_CONTROL_WORD_FAULT_RESET       0x0080U      /**< 故障复位，bit7 上升沿有效，对应手册 4.1.1 状态切换 15。 */
#define MID_LISAN_EC_CONTROL_WORD_HALT              0x010FU      /**< 减速停止，由运行命令 0Fh 叠加 bit8 组成，对应手册 4.2.2「运行」表的 010Fh。 */
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

MID_CANopenStatusTypeDef MID_LisanEc_EnableDrive(uint8_t nodeId);

MID_CANopenStatusTypeDef MID_LisanEc_WriteControlWord(uint8_t nodeId, uint16_t controlWord);

MID_CANopenStatusTypeDef MID_LisanEc_ReadControlWord(uint8_t nodeId, uint16_t *pControlWord);

MID_CANopenStatusTypeDef MID_LisanEc_SetMotorFree(uint8_t nodeId);

MID_CANopenStatusTypeDef MID_LisanEc_SetMotorLock(uint8_t nodeId);

MID_CANopenStatusTypeDef MID_LisanEc_QuickStop(uint8_t nodeId);

MID_CANopenStatusTypeDef MID_LisanEc_Halt(uint8_t nodeId);

MID_CANopenStatusTypeDef MID_LisanEc_FaultReset(uint8_t nodeId);

MID_CANopenStatusTypeDef MID_LisanEc_SetOperationMode(uint8_t nodeId, MID_LisanEcModeTypeDef mode);

MID_CANopenStatusTypeDef MID_LisanEc_ReadOperationMode(uint8_t nodeId, MID_LisanEcModeTypeDef *pMode);

MID_CANopenStatusTypeDef MID_LisanEc_SetMicroStep(uint8_t nodeId, uint32_t pulsePerRound);

MID_CANopenStatusTypeDef MID_LisanEc_SetPeakCurrent(uint8_t nodeId, uint16_t currentMa);

MID_CANopenStatusTypeDef MID_LisanEc_SetSmoothTime(uint8_t nodeId, uint16_t smoothTimeMs);

MID_CANopenStatusTypeDef MID_LisanEc_SetMinimumCurrent(uint8_t nodeId, uint16_t minimumCurrent);

MID_CANopenStatusTypeDef MID_LisanEc_SetMotorDirection(uint8_t nodeId, uint16_t direction);

MID_CANopenStatusTypeDef MID_LisanEc_SetProfileAcceleration(uint8_t nodeId, uint32_t acceleration);

MID_CANopenStatusTypeDef MID_LisanEc_SetProfileDeceleration(uint8_t nodeId, uint32_t deceleration);

MID_CANopenStatusTypeDef MID_LisanEc_SetProfileVelocity(uint8_t nodeId, uint32_t velocity);

MID_CANopenStatusTypeDef MID_LisanEc_SetTargetPosition(uint8_t nodeId, int32_t position);

MID_CANopenStatusTypeDef MID_LisanEc_SetTargetVelocity(uint8_t nodeId, int32_t velocity);

MID_CANopenStatusTypeDef MID_LisanEc_SetTargetTorque(uint8_t nodeId, int16_t torque);

MID_CANopenStatusTypeDef MID_LisanEc_SetMaxTorque(uint8_t nodeId, uint16_t maxTorque);

MID_CANopenStatusTypeDef MID_LisanEc_ConfigureTorqueMode(uint8_t nodeId, MID_LisanEcTorqueModeTypeDef torqueMode,
                                                         uint16_t offsetPulse, uint16_t maxSpeed);

MID_CANopenStatusTypeDef MID_LisanEc_ConfigureHoming(uint8_t nodeId, uint8_t method, uint32_t fastSpeed,
                                                     uint32_t slowSpeed, uint32_t acceleration);

MID_CANopenStatusTypeDef MID_LisanEc_MoveAbsolute(uint8_t nodeId, int32_t position, bool isImmediate);

MID_CANopenStatusTypeDef MID_LisanEc_MoveRelative(uint8_t nodeId, int32_t deltaPosition, bool isImmediate);

MID_CANopenStatusTypeDef MID_LisanEc_ClearSetpoint(uint8_t nodeId);

MID_CANopenStatusTypeDef MID_LisanEc_StartHoming(uint8_t nodeId);

MID_CANopenStatusTypeDef MID_LisanEc_ReadStatusWord(uint8_t nodeId, MID_LisanEcStatusWordTypeDef *pStatus);

MID_CANopenStatusTypeDef MID_LisanEc_ReadPosition(uint8_t nodeId, int32_t *pPosition);

MID_CANopenStatusTypeDef MID_LisanEc_ReadVelocity(uint8_t nodeId, int32_t *pVelocity);

MID_CANopenStatusTypeDef MID_LisanEc_ReadCurrent(uint8_t nodeId, int16_t *pCurrent);

MID_CANopenStatusTypeDef MID_LisanEc_ReadErrorCode(uint8_t nodeId, uint16_t *pErrorCode);

MID_CANopenStatusTypeDef MID_LisanEc_ReadDigitalInputs(uint8_t nodeId, uint32_t *pInputs);

MID_CANopenStatusTypeDef MID_LisanEc_SetNodeAddress(uint8_t nodeId, uint16_t newAddress, bool isFromObject);

MID_CANopenStatusTypeDef MID_LisanEc_StoreParameters(uint8_t nodeId);

MID_CANopenStatusTypeDef MID_LisanEc_RestoreDefault(uint8_t nodeId);

MID_LisanEcStateTypeDef MID_LisanEc_ParseState(uint16_t statusWord);

void MID_LisanEc_ParseStatusWord(uint16_t statusWord, MID_LisanEcStatusWordTypeDef *pStatus);

const char *MID_LisanEc_ErrorCodeToString(uint16_t errorCode);

#ifdef __cplusplus
}
#endif

#endif /* __MID_LISAN_EC_H */
