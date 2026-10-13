# AGENTS.md

> 本文件是本嵌入式项目的工程协作规范，适用于人类开发者、代码审查者以及 AI 编码助手。
> 目标：统一风格、降低沟通成本、提高可维护性、可测试性、可移植性和工程化水平。
> 所有新增或修改的 C/C++、头文件、构建脚本、文档、配置，默认都必须遵守本规范。

---

## 1. 项目技术基线

- 编译器：Arm Compiler 6，即 AC6。
- 语言标准：C99 或 C11，优先 C11；禁止使用可变长数组 VLA。
- 基础类型：统一使用 `stdint.h`、`stdbool.h`、`stddef.h`。
- 浮点使用：仅在必要模块使用 `float/double`，并明确单位与精度。
- 编译警告：开启 `-Wall -Wextra`，禁止提交有警告代码。
- 格式化工具：统一使用 `clang-format`，仓库根目录提供 `.clang-format`。
- 静态检查：建议启用 `clang-tidy`、`cppcheck`。
- 版本管理：Git，提交信息使用语义化前缀，如 `feat:`、`fix:`、`docs:`、`refactor:`、`test:`、`chore:`。

---

## 2. 缩进与格式规范：仿 HAL 库

### 2.1 缩进

- 统一使用 2 个空格缩进。
- 禁止使用 Tab。
- 预处理指令 `#` 顶格，内部代码按 2 空格缩进。
- 结构体成员、枚举成员、联合体成员按 2 空格缩进。
- `switch` 中 `case` 相对 `switch` 缩进 2 空格，`case` 内代码再缩进 2 空格。

```c
switch (s_appState)
{
  case APP_STATE_IDLE:
    break;
  case APP_STATE_RUN:
    break;
  default:
    s_appState = APP_STATE_ERROR;
    break;
}
```

### 2.2 大括号

- 使用 Allman 风格。
- 函数定义、`if`、`else`、`for`、`while`、`switch`、`do` 的左大括号独占一行。
- `else` 与前后大括号保持同一缩进层级。

```c
HAL_StatusTypeDef APP_Led_Init(void)
{
  if (s_ledState == GPIO_PIN_RESET)
  {
    s_ledState = GPIO_PIN_SET;
  }
  else
  {
    s_ledState = GPIO_PIN_RESET;
  }

  return HAL_OK;
}
```

### 2.3 空格与换行

- 关键字与左括号之间保留一个空格，如 `if (`、`for (`、`while (`、`switch (`。
- 函数名与左括号之间不留空格，如 `APP_Led_Init(`。
- 逗号后保留一个空格，分号前不留空格。
- 二元运算符两侧保留空格。
- 一元运算符与操作数之间不留空格。
- 类型转换右括号后保留一个空格，如 `(uint32_t)value`。
- 指针星号紧贴变量名，如 `UART_HandleTypeDef *huart`。
- 每行不超过 120 字符。
- 文件末尾保留一个换行。
- 禁止行尾空格。
- 函数之间空一行。
- 逻辑块之间可空一行。

### 2.4 多行参数

函数参数过多时，每行一个参数，并与第一个参数左对齐。
如果对齐后过长，则统一缩进 2 空格。

```c
HAL_StatusTypeDef APP_Comm_SendFrame(const uint8_t *pData,
                                     uint16_t length,
                                     uint32_t timeoutMs);
```

### 2.5 clang-format 配置

仓库根目录 `.clang-format` 必须与以下配置保持一致。

```yaml
BasedOnStyle: LLVM
IndentWidth: 2
TabWidth: 2
UseTab: Never
ColumnLimit: 120
BreakBeforeBraces: Allman
IndentCaseLabels: true
AllowShortIfStatementsOnASingleLine: false
AllowShortFunctionsOnASingleLine: None
PointerAlignment: Right
AlignConsecutiveDeclarations: true
AlignConsecutiveAssignments: false
SpaceAfterCStyleCast: true
```

---

## 3. 命名规范：采用类 HAL 库风格

### 3.1 文件命名

- 应用层：`app_xxx.c` / `app_xxx.h`
- 中间层：`mid_xxx.c` / `mid_xxx.h`
- 板级层：`bsp_xxx.c` / `bsp_xxx.h`
- 驱动层：`drv_xxx.c` / `drv_xxx.h`
- 配置文件：`xxx_config.h`
- 测试文件：`test_xxx.c`

示例：

```text
app_led.c
app_led.h
bsp_uart.c
bsp_uart.h
drv_oled.c
drv_oled.h
```

### 3.2 类型命名

使用大驼峰，并以 `TypeDef` 结尾，模仿 HAL。

```c
typedef struct {
    uint32_t BaudRate;
    uint32_t WordLength;
    uint32_t StopBits;
} APP_UartConfigTypeDef;

typedef enum {
    APP_STATE_IDLE = 0,
    APP_STATE_RUN,
    APP_STATE_ERROR
} APP_StateTypeDef;

typedef struct {
    UART_HandleTypeDef huart;
    uint8_t rxBuffer[APP_UART_RX_BUFFER_SIZE];
} APP_UartHandleTypeDef;
```

### 3.3 函数命名

格式：`模块_动作_对象()`，使用大驼峰，模块前缀全大写。

```c
HAL_StatusTypeDef BSP_Led_Init(void);
HAL_StatusTypeDef BSP_Led_On(void);
HAL_StatusTypeDef BSP_Led_Off(void);
HAL_StatusTypeDef BSP_Led_Toggle(void);

HAL_StatusTypeDef APP_Comm_SendFrame(const uint8_t *pData, uint16_t length);
```

私有函数必须加 `static`。

```c
static void MX_GPIO_Init(void);
static HAL_StatusTypeDef APP_Comm_ParseFrame(const uint8_t *pData, uint16_t length);
```

### 3.4 变量命名

- 局部变量：小驼峰，见名知意。
- 全局变量：`g_` 前缀，如 `g_appState`。
- 静态变量：`s_` 前缀，如 `s_uartHandle`。
- 指针：`p` 前缀，如 `pData`、`pFrame`。
- 句柄：`h` 前缀，如 `huart1`、`htim2`；指向 HAL 句柄的指针也一律用 `h`，如 `CAN_HandleTypeDef *hcan`、`UART_HandleTypeDef *huart`、`DMA_HandleTypeDef *hdma`，禁止写成 `pHcan`、`pUart`、`pHdma`。
- RTOS 句柄（`osSemaphoreId_t`、`osThreadId_t`、`osMessageQueueId_t`、`TaskHandle_t` 等）同样用 `h`，静态句柄为 `s_h` 开头，如 `s_hRxSemaphore`；只有控制块与属性结构体沿用 `s_` 前缀，如 `s_rxSemaphoreBuffer`、`s_rxSemaphoreAttr`。
- 布尔：`is`、`has`、`can` 开头，如 `isReady`、`hasError`。
- 常量：优先 `#define` 或 `const`，全大写或大驼峰，带单位。

```c
static UART_HandleTypeDef s_huart1;
static APP_StateTypeDef s_appState = APP_STATE_IDLE;
uint32_t uartBaudRate = 115200U;
const uint32_t APP_LED_BLINK_PERIOD_MS = 500U;
```

### 3.5 变量取值模仿 HAL：使用 Doxygen group 组织

所有枚举、宏、取值范围，必须放入 `@defgroup` 中，取值风格模仿 HAL，形如 `<模块>_<属性>_<取值>`，并使用带 `U` 后缀的十六进制或十进制常量。

```c
/**
  * @defgroup APP_UART_Word_Length APP UART Word Length
  * @{
  */
#define APP_UART_WORDLENGTH_8B   0x00000000U
#define APP_UART_WORDLENGTH_9B   0x00001000U
/**
  * @}
  */

/**
  * @defgroup APP_UART_Stop_Bits APP UART Stop Bits
  * @{
  */
#define APP_UART_STOPBITS_1      0x00000000U
#define APP_UART_STOPBITS_2      0x00002000U
/**
  * @}
  */

/**
  * @defgroup APP_LED_State APP LED State
  * @{
  */
#define APP_LED_STATE_OFF        0x00000000U
#define APP_LED_STATE_ON         0x00000001U
/**
  * @}
  */
```

枚举同样放入 `@defgroup`。

```c
/**
  * @defgroup APP_Comm_Status APP Communication Status
  * @{
  */
typedef enum {
    APP_COMM_STATUS_IDLE  = 0x00U,
    APP_COMM_STATUS_BUSY  = 0x01U,
    APP_COMM_STATUS_ERROR = 0x02U
} APP_CommStatusTypeDef;
/**
  * @}
  */
```

### 3.6 宏命名

全大写，下划线分隔，模块前缀明确，带单位。

```c
/**
  * @defgroup APP_UART_Config APP UART Configuration
  * @{
  */
#define APP_UART_BAUDRATE          115200U
#define APP_UART_RX_BUFFER_SIZE    256U
/**
  * @}
  */

/**
  * @defgroup APP_Task_Period APP Task Period
  * @{
  */
#define APP_TASK_PERIOD_MS         10U
#define APP_LED_BLINK_PERIOD_MS    500U
#define APP_MAX_RETRY_COUNT        3U
/**
  * @}
  */
```

禁止：

```c
#define SIZE 10
#define DELAY 500
#define UART_BAUD 115200
```

---

## 4. 注释规范：Doxygen + 类 HAL 工程注释

注释语言：中文优先。
禁止在注释中使用任何 emoji。
禁止在注释中描述与上次提交的区别。
注释只解释做了什么，以及为什么这么做。
尽可能减少行间注释，能通过命名或代码结构表达清楚的，不再写注释。
避免在注释中使用括号进行补充解释。

### 4.1 文件头注释

每个 `.c` / `.h` 文件必须有文件头，采用 HAL 风格星号框。
文件头必须包含版权声明和 license，取自仓库根目录 LICENSE 文件与 git 提交信息中的作者信息。
`@author` 填写 git 提交信息中的作者名。

```c
/**
  ******************************************************************************
  * @file    app_led.c
  * @brief   LED 应用模块。
  * @author  Your Name
  * @date    2026-01-01
  * @version V1.0.0
  ******************************************************************************
  * @attention
  *
  * Copyright 2026 Your Team.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  *
  ******************************************************************************
  */
```

### 4.2 函数注释

所有对外函数、重要私有函数必须使用 Doxygen。
函数 Doxygen 注释一律写在 `.c` 的函数定义处，`.h` 中只保留函数声明，不重复放置函数注释。
类型、宏、枚举的注释仍随其定义放在 `.h`。

注释必须说明函数实际做了什么，禁止同义反复复述函数名。
`@brief` 用一句话概括职责；凡是有流程、有分支或有副作用的函数，都要用 `@note` 写清关键步骤与处理顺序、
写入了哪个对象或寄存器、失败时是否继续、门限与限幅取自哪个宏。
`@brief 底盘链路任务`、`@brief 注册从站节点` 这类不含新增信息的写法一律不接受。

`.h` 只放声明：

```c
HAL_StatusTypeDef BSP_Led_Init(void);
HAL_StatusTypeDef APP_Comm_SendFrame(const uint8_t *pData, uint16_t length);
```

`.c` 在函数定义处写 Doxygen：

```c
/**
  * @brief  初始化 LED 控制模块。
  * @param  None
  * @retval HAL_OK     初始化成功。
  * @retval HAL_ERROR  初始化失败。
  * @note   依次使能 GPIO 时钟、把引脚配置为推挽输出并置为熄灭电平；重复调用会重新配置引脚。
  * @warning 该函数不可在中断中调用。
  */
HAL_StatusTypeDef BSP_Led_Init(void)
{
  /* ... */
}

/**
  * @brief  发送一帧数据。
  * @param  pData   指向待发送数据缓冲区。
  * @param  length  数据长度，单位字节。
  * @retval HAL_OK     发送成功。
  * @retval HAL_ERROR  参数错误或发送失败。
  * @retval HAL_BUSY   外设忙。
  * @note   先校验指针与长度，再把数据填入发送缓冲并启动发送；
  *         上一帧未发完时直接返回 HAL_BUSY，调用方需自行重试。
  */
HAL_StatusTypeDef APP_Comm_SendFrame(const uint8_t *pData, uint16_t length)
{
  /* ... */
}
```

### 4.3 结构体与枚举注释

```c
/**
  * @brief UART 配置结构体。
  */
typedef struct {
    uint32_t BaudRate;   /**< 波特率，单位 bps。 */
    uint32_t WordLength; /**< 数据位。 */
    uint32_t StopBits;   /**< 停止位。 */
} APP_UartConfigTypeDef;
```

禁止无意义注释，如 `i++; /* i 加 1 */`。

---

## 5. 减少 Magic Number

禁止在业务代码中出现无说明的裸数字。以下情况可例外：

- `0`、`1`、`-1` 在明确语义下可使用。
- 循环下标 `i = 0`。
- 明显布尔判断。

其他数字必须定义为宏、枚举或 `const` 常量，并按 `@defgroup` 组织。

所有可调参数放入 `config/`。

```c
/* config/app_config.h */

/**
  * @defgroup APP_UART_Config APP UART Configuration
  * @{
  */
#define APP_UART_BAUDRATE          115200U
#define APP_UART_RX_BUFFER_SIZE    256U
/**
  * @}
  */

/**
  * @defgroup APP_Timing_Config APP Timing Configuration
  * @{
  */
#define APP_TASK_PERIOD_MS         10U
#define APP_LED_BLINK_PERIOD_MS    500U
#define APP_TIMEOUT_MS             1000U
#define APP_SAMPLE_PERIOD_US       100U
/**
  * @}
  */

/**
  * @defgroup APP_Buffer_Size APP Buffer Size
  * @{
  */
#define APP_BUFFER_SIZE_BYTES      128U
/**
  * @}
  */
```

位操作使用带 `U` 后缀的移位常量。

```c
/**
  * @defgroup APP_Status_Bit APP Status Bit
  * @{
  */
#define APP_STATUS_LED_ON          (1U << 0)
#define APP_STATUS_UART_READY      (1U << 1)
#define APP_STATUS_ERROR           (1U << 2)
/**
  * @}
  */
```

数组长度使用统一宏。

```c
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
```

---

## 6. 工程化增强要求

### 6.1 错误处理

对外函数统一返回状态，优先 `HAL_StatusTypeDef` 或项目统一 `APP_StatusTypeDef`。
禁止忽略返回值。
参数检查放在函数入口。
错误码必须可读，禁止返回裸 `-1`。

```c
typedef enum {
    APP_OK = 0x00U,
    APP_ERROR = 0x01U,
    APP_BUSY = 0x02U,
    APP_TIMEOUT = 0x03U,
    APP_INVALID_PARAM = 0x04U
} APP_StatusTypeDef;
```

### 6.2 日志与调试

统一日志接口，禁止直接使用 `printf` 散落各层。

```c
LOG_DEBUG("uart rx len=%u", length);
LOG_INFO("app started");
LOG_WARN("retry count=%u", retryCount);
LOG_ERROR("uart init failed");
```

- 中断中禁止阻塞式打印。
- 日志可通过宏开关裁剪。
- 日志级别：DEBUG / INFO / WARN / ERROR。

### 6.3 断言

```c
#define APP_ASSERT(expr) do { \
    if (!(expr)) { \
        APP_AssertFailed(__FILE__, __LINE__); \
        while (1) { } \
    } \
} while (0)
```

### 6.4 中断规范

- 中断服务函数尽量短。
- 中断中只做置标志、入队、清中断。
- 共享变量必须 `volatile`，必要时使用临界区。
- 禁止在中断中动态分配内存、阻塞延时、复杂打印。

### 6.5 时间与任务

- 使用非阻塞方式，状态机配合 `HAL_GetTick()`。
- 禁止长时间 `HAL_Delay()` 阻塞主循环。
- 周期任务使用统一时间基准。
- 超时判断必须使用带单位常量。

### 6.6 状态机

```c
switch (s_appState)
{
  case APP_STATE_IDLE:
    break;
  case APP_STATE_RUN:
    break;
  case APP_STATE_ERROR:
    break;
  default:
    s_appState = APP_STATE_ERROR;
    break;
}
```

### 6.7 内存管理

- 嵌入式项目默认禁止动态内存分配。
- 如必须使用，统一封装，并限制最大堆大小。
- 优先静态分配、内存池、环形缓冲区。
- 缓冲区必须检查边界。

### 6.8 头文件规范

头文件必须使用类 HAL 风格的预编译保护，格式为 `__<模块>_<文件>_H`。
头文件只放声明，不放变量定义。
对外暴露最小接口。
include 顺序：对应头文件、C 标准库、第三方库、项目内部头文件。

```c
#ifndef __APP_LED_H
#define __APP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

HAL_StatusTypeDef APP_Led_Init(void);
HAL_StatusTypeDef APP_Led_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_LED_H */
```

### 6.9 版本与文档

- 使用语义化版本：`MAJOR.MINOR.PATCH`。
- 每个模块有设计说明或 README。
- 接口变更必须同步更新注释和文档。

### 6.10 测试

- 核心算法、协议解析、状态机必须有单元测试。
- 使用 Unity、CMock 或类似框架。
- 对 HAL 层进行 Mock，避免测试依赖硬件。
- CI 中至少执行：构建、静态检查、单元测试。

### 6.11 Git 协作

- 主分支保护，禁止直接 push。
- 功能分支：`feature/xxx`、`fix/xxx`。
- PR 必须通过 CI 和至少一人审查。
- 提交信息使用语义化前缀，冒号后使用中文。

```text
feat: 添加 UART DMA 接收支持
fix: 修正 LED 闪烁周期计算错误
docs: 更新编码规范
refactor: 拆分包解析函数
test: 补充 UART 状态机单元测试
chore: 更新静态检查脚本
```

### 6.12 代码审查清单

- 是否符合命名规范。
- 是否符合缩进与格式规范。
- 是否有 Doxygen 注释。
- 是否存在 Magic Number。
- 是否检查返回值。
- 是否有边界检查。
- 是否违反分层调用。
- 中断是否足够短。
- 是否有阻塞延时。
- 是否引入动态内存。
- 是否有单元测试或测试说明。
- 是否通过编译、格式化、静态检查。

---

## 7. AI 编码助手要求

AI 生成代码时必须：

1. 先阅读本 `AGENTS.md`。
2. 遵循 HAL 风格命名、缩进与 Doxygen 注释；函数 Doxygen 写在 `.c` 的函数定义处，`.h` 只保留声明。
3. 不编造不存在的 HAL API。
4. 不引入未声明的依赖。
5. 不使用 Magic Number，取值放入 `@defgroup`。
6. 对外函数必须返回统一状态。
7. 头文件使用 `__<模块>_<文件>_H` 形式的预编译保护。
8. 注释不使用 emoji，不描述与上次提交的差异，只说明做了什么和为什么。
9. 缩进使用 2 空格，禁止 Tab，大括号使用 Allman 风格。
10. 修改后给出影响范围、测试建议和审查要点。
11. 如果需求不明确，先提出最小必要问题，不要擅自假设硬件细节。

---

## 8. 示例：完整模块风格

```c
/**
  ******************************************************************************
  * @file    app_led.h
  * @brief   LED 应用模块接口。
  * @author  Your Name
  * @date    2026-01-01
  * @version V1.0.0
  ******************************************************************************
  * @attention
  *
  * Copyright 2026 Your Team.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  *
  ******************************************************************************
  */

#ifndef __APP_LED_H
#define __APP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

HAL_StatusTypeDef APP_Led_Init(void);
HAL_StatusTypeDef APP_Led_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_LED_H */
```

```c
/**
  ******************************************************************************
  * @file    app_led.c
  * @brief   LED 应用模块实现。
  * @author  Your Name
  * @date    2026-01-01
  * @version V1.0.0
  ******************************************************************************
  * @attention
  *
  * Copyright 2026 Your Team.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  *
  ******************************************************************************
  */

#include "app_led.h"
#include "app_config.h"

static uint32_t s_lastToggleTick = 0U;
static GPIO_PinState s_ledState = GPIO_PIN_RESET;

/**
  * @brief  初始化 LED 应用模块。
  * @param  None
  * @retval HAL_OK     初始化成功。
  * @retval HAL_ERROR  初始化失败。
  */
HAL_StatusTypeDef APP_Led_Init(void)
{
  s_lastToggleTick = HAL_GetTick();
  s_ledState = GPIO_PIN_RESET;

  if (HAL_GPIO_WritePin(APP_LED_GPIO_PORT, APP_LED_GPIO_PIN, s_ledState) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  周期处理 LED 闪烁逻辑。
  * @param  None
  * @retval None
  * @note   需在主循环或周期任务中调用。
  */
void APP_Led_Process(void)
{
  uint32_t currentTick = HAL_GetTick();

  if ((currentTick - s_lastToggleTick) >= APP_LED_BLINK_PERIOD_MS)
  {
    s_lastToggleTick = currentTick;
    s_ledState = (s_ledState == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(APP_LED_GPIO_PORT, APP_LED_GPIO_PIN, s_ledState);
  }
}
```

---

## 9. 最终原则

- 代码首先是给人看的，其次才是给机器执行的。
- 命名即文档，注释解释为什么，而不是重复做了什么。
- 配置集中、接口清晰、分层严格、错误可追踪。
- 任何临时代码都必须有明确 TODO 和负责人。
- 不确定时，优先选择更简单、更可测试、更易维护的方案。
