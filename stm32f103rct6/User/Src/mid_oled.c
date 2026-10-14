/**
  ******************************************************************************
  * @file    mid_oled.c
  * @brief   SSD1306 128x32 点阵屏驱动实现，软件 I2C 加页地址帧缓冲。
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

#include "mid_oled.h"
#include "mid_oled_font.h"

#include <stddef.h>
#include <string.h>

#include "main.h"

#define MID_OLED_BIT_DELAY_LOOPS    26U   /**< 软件 I2C 半周期延时循环次数，按 72 MHz 主频标定。 */
#define MID_OLED_STARTUP_DELAY_MS   100U  /**< 上电后等待屏幕内部复位完成的时长，单位毫秒。 */

#define MID_OLED_CMD_DISPLAY_OFF    0xAEU /**< 关闭显示输出。 */
#define MID_OLED_CMD_CLOCK_DIV      0xD5U /**< 设置时钟分频与振荡频率。 */
#define MID_OLED_CMD_MULTIPLEX      0xA8U /**< 设置驱动路数，值等于行数减一。 */
#define MID_OLED_CMD_CHARGE_PUMP    0x8DU /**< 电荷泵配置入口。 */
#define MID_OLED_CMD_MEMORY_MODE    0x20U /**< 显存寻址模式入口。 */
#define MID_OLED_CMD_SEG_REMAP      0xA1U /**< 段重映射，左右镜像。 */
#define MID_OLED_CMD_COM_SCAN_DEC   0xC8U /**< 行扫描方向反向。 */
#define MID_OLED_CMD_COM_PINS       0xDAU /**< 行硬件配置入口。 */
#define MID_OLED_CMD_CONTRAST       0x81U /**< 对比度入口。 */
#define MID_OLED_CMD_PRECHARGE      0xD9U /**< 预充电周期入口。 */
#define MID_OLED_CMD_VCOMH          0xDBU /**< 行截止电压入口。 */
#define MID_OLED_CMD_DISPLAY_ON     0xAFU /**< 打开显示输出。 */
#define MID_OLED_CMD_PAGE_BASE      0xB0U /**< 页地址基数，或上页号得到目标页。 */
#define MID_OLED_CMD_COLUMN_LOW     0x00U /**< 列地址低四位基数。 */
#define MID_OLED_CMD_COLUMN_HIGH    0x10U /**< 列地址高四位数基数。 */
#define MID_OLED_CMD_START_LINE     0x40U /**< 显示起始行基数，或上行号。 */
#define MID_OLED_CMD_DISPLAY_NORMAL 0xA6U /**< 正常显示，非反色。 */
#define MID_OLED_CMD_DISPLAY_OFFSET 0xD3U /**< 显示垂直偏移入口。 */

static uint8_t s_frameBuffer[MID_OLED_FRAME_BYTES]; /**< 帧缓冲，按页连续存放，一字节一列。 */
static uint8_t s_slaveAddress = (uint8_t)(MID_OLED_I2C_ADDRESS << 1U); /**< 实际使用的 8 位从机地址，初始化时探测确定。 */

static void MID_Oled_ResolveAddress(void);
static void MID_Oled_GpioInit(void);
static void MID_Oled_BitDelay(void);
static void MID_Oled_I2cStart(void);
static void MID_Oled_I2cStop(void);
static void MID_Oled_I2cWriteByte(uint8_t byte, bool *pAck);
static void MID_Oled_WriteCommand(uint8_t command);
static void MID_Oled_WriteData(const uint8_t *pData, uint16_t length);
static void MID_Oled_SetPosition(uint8_t column, uint8_t page);

/**
  * @brief  软件 I2C 半周期延时。
  * @param  None
  * @retval None
  * @note   用空操作循环近似，循环次数按 72 MHz 主频标定到约 2 微秒，
  *         对应约 250 kHz 位速率；改主频需同步调整 MID_OLED_BIT_DELAY_LOOPS。
  */
static void MID_Oled_BitDelay(void)
{
  volatile uint32_t loops = MID_OLED_BIT_DELAY_LOOPS;

  while (loops > 0U)
  {
    loops--;
  }
}

/**
  * @brief  发送 I2C 起始位。
  * @param  None
  * @retval None
  * @note   时钟为高时数据由高拉低即产生起始位，之后拉低时钟准备送数据。
  */
static void MID_Oled_I2cStart(void)
{
  HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET);
  MID_Oled_BitDelay();
  HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_RESET);
  MID_Oled_BitDelay();
  HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  发送 I2C 停止位。
  * @param  None
  * @retval None
  * @note   先拉低数据再拉高时钟，时钟为高时数据由低拉高即产生停止位。
  */
static void MID_Oled_I2cStop(void)
{
  HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET);
  MID_Oled_BitDelay();
  HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET);
  MID_Oled_BitDelay();
}

/**
  * @brief  按高位在前发送一个字节，随后产生第 9 个时钟并读取从机应答。
  * @param  byte  待发送字节。
  * @param  pAck  输出参数，收到低电平应答即置真，不可为 NULL。
  * @retval None
  * @note   数据在时钟低电平时改变、高电平时被采样，连续 8 个时钟发出一个字节；
  *         第 9 个时钟是应答时钟，主机必须先释放 SDA（本端口为开漏输出，写高即由外部上拉拉高），
  *         再拉高 SCL，此时从机把 SDA 拉低表示应答。SSD1306 依赖该时钟推进字节计数，
  *         缺少它会导致控制字节与命令错位、屏幕无显示。
  */
static void MID_Oled_I2cWriteByte(uint8_t byte, bool *pAck)
{
  uint8_t bitIndex = 0U;

  for (bitIndex = 0U; bitIndex < 8U; bitIndex++)
  {
    HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin,
                      ((byte & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    byte = (uint8_t)(byte << 1U);
    MID_Oled_BitDelay();
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET);
    MID_Oled_BitDelay();
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET);
    MID_Oled_BitDelay();
  }

  /* 第 9 个时钟：先释放 SDA，再脉动 SCL，从机在 SCL 为高期间把 SDA 拉低表示应答。 */
  HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET);
  MID_Oled_BitDelay();
  HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET);
  MID_Oled_BitDelay();
  *pAck = (HAL_GPIO_ReadPin(IIC_SDA_GPIO_Port, IIC_SDA_Pin) == GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET);
  MID_Oled_BitDelay();
}

/**
  * @brief  向屏幕写入一条命令。
  * @param  command 命令字节。
  * @retval None
  * @note   命令字节前要送控制字节 0x00，表示后续为命令流。
  */
static void MID_Oled_WriteCommand(uint8_t command)
{
  bool ack = false;

  MID_Oled_I2cStart();
  MID_Oled_I2cWriteByte(s_slaveAddress, &ack);
  MID_Oled_I2cWriteByte(0x00U, &ack);
  MID_Oled_I2cWriteByte(command, &ack);
  MID_Oled_I2cStop();
}

/**
  * @brief  向屏幕写入一段显示数据。
  * @param  pData   数据首地址，不可为 NULL。
  * @param  length  数据长度，单位字节。
  * @retval None
  * @note   数据流前送控制字节 0x40，后续字节连续写入当前列地址，列地址自动递增。
  */
static void MID_Oled_WriteData(const uint8_t *pData, uint16_t length)
{
  bool     ack = false;
  uint16_t index = 0U;

  MID_Oled_I2cStart();
  MID_Oled_I2cWriteByte(s_slaveAddress, &ack);
  MID_Oled_I2cWriteByte(0x40U, &ack);

  for (index = 0U; index < length; index++)
  {
    MID_Oled_I2cWriteByte(pData[index], &ack);
  }

  MID_Oled_I2cStop();
}

/**
  * @brief  把列地址与页地址送到屏幕。
  * @param  column 目标列，取值范围 0 至 MID_OLED_COLUMNS 减 1。
  * @param  page   目标页，取值范围 0 至 MID_OLED_PAGES 减 1。
  * @retval None
  * @note   列地址按高低四位分两次送，页地址用 0xB0 或页号。
  */
static void MID_Oled_SetPosition(uint8_t column, uint8_t page)
{
  MID_Oled_WriteCommand((uint8_t)(MID_OLED_CMD_PAGE_BASE | (page & 0x03U)));
  MID_Oled_WriteCommand((uint8_t)(MID_OLED_CMD_COLUMN_LOW | (column & 0x0FU)));
  MID_Oled_WriteCommand((uint8_t)(MID_OLED_CMD_COLUMN_HIGH | ((column >> 4U) & 0x0FU)));
}

/**
  * @brief  配置软件 I2C 引脚，并给 SDA 打开内部上拉。
  * @param  None
  * @retval None
  * @note   SDA 为开漏输出，输出高时只是释放总线，靠上拉电阻把线拉高；若屏幕模块没装外部上拉，
  *         必须打开内部上拉，否则起始位无法建立、从机不应答、屏幕一直黑。SCL 为推挽输出无需上拉。
  *         在 MX_GPIO_Init 之后调用，覆盖其 NOPULL 配置。
  */
static void MID_Oled_GpioInit(void)
{
  GPIO_InitTypeDef gpioInit = {0};

  gpioInit.Pin = IIC_SDA_Pin;
  gpioInit.Mode = GPIO_MODE_OUTPUT_OD;
  gpioInit.Pull = GPIO_PULLUP;
  gpioInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(IIC_SDA_GPIO_Port, &gpioInit);

  gpioInit.Pin = IIC_SCL_Pin;
  gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(IIC_SCL_GPIO_Port, &gpioInit);

  HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET);
}

/**
  * @brief  探测屏幕实际应答的从机地址。
  * @param  None
  * @retval None
  * @note   先试 0x3C（SA0 接地），无应答再试 0x3D（SA0 接高），两者都无应答时保持默认 0x3C。
  *         不同批次模块的 SA0 接法不同，自动探测可免去逐块改宏。
  */
static void MID_Oled_ResolveAddress(void)
{
  bool ack = false;

  MID_Oled_I2cStart();
  MID_Oled_I2cWriteByte((uint8_t)(MID_OLED_I2C_ADDRESS << 1U), &ack);
  MID_Oled_I2cStop();
  if (ack != false)
  {
    s_slaveAddress = (uint8_t)(MID_OLED_I2C_ADDRESS << 1U);
    return;
  }

  MID_Oled_I2cStart();
  MID_Oled_I2cWriteByte((uint8_t)(MID_OLED_I2C_ADDRESS_ALT << 1U), &ack);
  MID_Oled_I2cStop();
  if (ack != false)
  {
    s_slaveAddress = (uint8_t)(MID_OLED_I2C_ADDRESS_ALT << 1U);
    return;
  }

  s_slaveAddress = (uint8_t)(MID_OLED_I2C_ADDRESS << 1U);
}

void MID_Oled_Init(void)
{
  MID_Oled_GpioInit();

  HAL_Delay(MID_OLED_STARTUP_DELAY_MS);

  MID_Oled_ResolveAddress();

  MID_Oled_WriteCommand(MID_OLED_CMD_DISPLAY_OFF);
  MID_Oled_WriteCommand((uint8_t)(MID_OLED_CMD_START_LINE | 0x00U));
  MID_Oled_WriteCommand((uint8_t)(MID_OLED_CMD_PAGE_BASE | 0x00U));
  MID_Oled_WriteCommand(MID_OLED_CMD_COM_SCAN_DEC);
  MID_Oled_WriteCommand(MID_OLED_CMD_CONTRAST);
  MID_Oled_WriteCommand(0xFFU);
  MID_Oled_WriteCommand(MID_OLED_CMD_SEG_REMAP);
  MID_Oled_WriteCommand(MID_OLED_CMD_DISPLAY_NORMAL);
  MID_Oled_WriteCommand(MID_OLED_CMD_MULTIPLEX);
  MID_Oled_WriteCommand((uint8_t)(MID_OLED_PAGES * MID_OLED_PAGE_ROWS - 1U));
  MID_Oled_WriteCommand(MID_OLED_CMD_DISPLAY_OFFSET);
  MID_Oled_WriteCommand(0x00U);
  MID_Oled_WriteCommand(MID_OLED_CMD_CLOCK_DIV);
  MID_Oled_WriteCommand(0xF0U);
  MID_Oled_WriteCommand(MID_OLED_CMD_PRECHARGE);
  MID_Oled_WriteCommand(0x22U);
  MID_Oled_WriteCommand(MID_OLED_CMD_COM_PINS);
  MID_Oled_WriteCommand(0x02U);
  MID_Oled_WriteCommand(MID_OLED_CMD_VCOMH);
  MID_Oled_WriteCommand(0x49U);
  MID_Oled_WriteCommand(MID_OLED_CMD_CHARGE_PUMP);
  MID_Oled_WriteCommand(0x14U);
  MID_Oled_WriteCommand(MID_OLED_CMD_MEMORY_MODE);
  MID_Oled_WriteCommand(0x02U);
  MID_Oled_WriteCommand(MID_OLED_CMD_DISPLAY_ON);

  (void)memset(s_frameBuffer, 0, sizeof(s_frameBuffer));
}

void MID_Oled_Flush(void)
{
  uint8_t page = 0U;

  for (page = 0U; page < MID_OLED_PAGES; page++)
  {
    MID_Oled_SetPosition(0U, page);
    MID_Oled_WriteData(&s_frameBuffer[(uint16_t)page * MID_OLED_COLUMNS], MID_OLED_COLUMNS);
  }
}

void MID_Oled_Clear(void)
{
  (void)memset(s_frameBuffer, 0, sizeof(s_frameBuffer));
}

void MID_Oled_SetByte(uint8_t column, uint8_t page, uint8_t value)
{
  if ((column >= MID_OLED_COLUMNS) || (page >= MID_OLED_PAGES))
  {
    return;
  }

  s_frameBuffer[((uint16_t)page * MID_OLED_COLUMNS) + column] = value;
}

void MID_Oled_DrawString(uint8_t column, uint8_t page, const char *pText)
{
  uint8_t glyph[MID_OLED_FONT_WIDTH_BYTES];
  uint8_t byteIndex = 0U;
  uint8_t charCode = 0U;

  if (pText == NULL)
  {
    return;
  }

  while (*pText != '\0')
  {
    charCode = (uint8_t)*pText;
    if ((charCode < MID_OLED_FONT_FIRST_CHAR) || (charCode > MID_OLED_FONT_LAST_CHAR))
    {
      charCode = ' ';
    }

    charCode = (uint8_t)(charCode - MID_OLED_FONT_FIRST_CHAR);
    (void)memcpy(glyph, &g_oledFont6x8[charCode][0], sizeof(glyph));

    for (byteIndex = 0U; byteIndex < MID_OLED_FONT_WIDTH_BYTES; byteIndex++)
    {
      MID_Oled_SetByte((uint8_t)(column + byteIndex), page, glyph[byteIndex]);
    }

    column = (uint8_t)(column + MID_OLED_FONT_WIDTH_BYTES);
    if (column >= MID_OLED_COLUMNS)
    {
      break;
    }

    pText++;
  }
}

void MID_Oled_DrawUint(uint8_t column, uint8_t page, uint32_t value, uint8_t minDigits)
{
  uint8_t  digits[10U];
  uint8_t  count = 0U;
  uint8_t  index = 0U;
  char     text[12U];
  uint8_t  textIndex = 0U;

  do
  {
    digits[count] = (uint8_t)(value % 10U);
    value /= 10U;
    count++;
  } while ((value != 0U) && (count < (uint8_t)sizeof(digits)));

  while ((count + textIndex) < minDigits)
  {
    text[textIndex] = '0';
    textIndex++;
  }

  for (index = count; index > 0U; index--)
  {
    text[textIndex] = (char)('0' + digits[index - 1U]);
    textIndex++;
  }

  text[textIndex] = '\0';
  MID_Oled_DrawString(column, page, text);
}

void MID_Oled_DrawBar(uint8_t column, uint8_t page, uint8_t width, uint16_t permille)
{
  uint8_t  index = 0U;
  uint16_t filled = 0U;

  if (permille > 1000U)
  {
    permille = 1000U;
  }

  if ((width == 0U) || (column >= MID_OLED_COLUMNS))
  {
    return;
  }

  if ((uint16_t)column + width > MID_OLED_COLUMNS)
  {
    width = (uint8_t)(MID_OLED_COLUMNS - column);
  }

  filled = (uint16_t)(((uint32_t)permille * (uint32_t)(width - 2U)) / 1000U);

  for (index = 0U; index < width; index++)
  {
    uint8_t rows = 0U;

    if ((index == 0U) || (index == (uint8_t)(width - 1U)))
    {
      rows = 0x80U;
    }
    else if (index <= filled)
    {
      rows = 0x7FU;
    }
    else
    {
      rows = 0x00U;
    }

    MID_Oled_SetByte((uint8_t)(column + index), page, rows);
  }
}
